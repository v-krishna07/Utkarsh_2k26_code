'use strict';
// ============================================================================
//  maze.js - Hidden-maze mapper module (pure functions + state, no I/O).
//
//  Like control.js: all maths lives here, the server only pipes data in/out.
//
//  Pipeline:
//    1. Sensor packets arrive (9-byte binary from ESP32) -> parseSensorPacket
//    2. Sliding median filter (5 readings per sensor) with close-range memory
//    3. Dead-reckoned pose from sent PWM (v, ω)
//    4. V_FULL auto-calibration from front wall readings
//    5. Heading correction from side-wall slope
//    6. Sideways position correction from (R - L)
//    7. Occupancy grid update (log-odds, ray-cast, persistent front blocked cell)
//    8. Graph / node updates (junctions, dead ends, no duplicates on U-turn)
//    9. Side opening tracking and junction re-anchoring
//   10. Turn log with node IDs
// ============================================================================

// ---- CAR & SENSOR CONFIGURATION -------------------------------------------
const MAZE_CONFIG = Object.freeze({
    CAR_WIDTH_CM:   14,
    CAR_LENGTH_CM:  16,
    // Sensor mounting offsets from car centre (cm).
    // Positive X = right, positive Y = forward.
    SENSOR_LEFT:   { x: -7.0, y: 0.0, angle_deg: -90 },   // points left
    SENSOR_FRONT:  { x:  0.0, y: 8.0, angle_deg:   0 },   // points forward
    SENSOR_RIGHT:  { x:  7.0, y: 0.0, angle_deg:  90 },   // points right
    // Acoustic constants
    SPEED_OF_SOUND_CM_PER_US: 0.0343,   // at ~20 °C
    MIN_DIST_CM:  2.5,
    MAX_DIST_CM: 100,
    FRONT_BLOCKED_CM: 12,        // blocked flag threshold
    ADVANCE_CM: 15,              // junction center advance (half lane width)
    AUTO_ADVANCE_SIM: true,      // simulator auto-advances on A/D at opening
    IS_SIMULATOR: false,
    CLOSE_MEMORY_MS: 500,        // remember close reading for 500 ms on no-echo
    CLOSE_THRESHOLD_CM: 20,      // readings under 20 cm trigger close-memory
    // Dead reckoning defaults (will be auto-calibrated)
    V_FULL_DEFAULT:    40,       // cm/s at PWM 255 (safe starting guess)
    SPIN_DEG_S_DEFAULT: 180,     // deg/s at differential 255 (safe guess)
    V_FULL_MIN:   5,
    V_FULL_MAX: 150,
    // Occupancy grid
    GRID_CELL_CM:  2,
    GRID_SIZE_CELLS: 400,        // 400 * 2 cm = 8 m
    LOG_ODD_FREE:   -0.4,
    LOG_ODD_OCC:     0.9,
    LOG_ODD_CLAMP:   5.0,
    LOG_ODD_NO_ECHO_FREE: -0.2,  // reduced confidence for no-echo beams
    LOG_ODD_SPIN_SCALE: 0.3,     // weight multiplier while spinning
    NO_ECHO_FREE_CM:  50,        // trust only 50 cm of free when no echo
    // Graph
    NODE_SPACING_CM:  15,
    NODE_MERGE_CM:     9,
    DEAD_END_FRONT_CM: 16,
    // Side openings
    OPENING_THRESHOLD_CM: 45,
    OPENING_HYSTERESIS_CM: 8,
    OPENING_ENTER_CM:  28,
    OPENING_DEDUP_CM:  20,
    // Heading correction
    BOTH_WALLS_CM:  40,
    HEADING_CORR_DIST_CM: 12,
    HEADING_CORR_MAX_DEG: 3,
    // V_FULL auto-tuning
    VFULL_FRONT_MIN_CM:  8,
    VFULL_FRONT_MAX_CM: 95,
    VFULL_WINDOW_CM:     6,
    VFULL_LEARN_RATE:    0.08,
    VFULL_RATIO_MIN:     0.4,
    VFULL_RATIO_MAX:     2.5,
    // Lane correction
    LANE_RESET_SPIN_DEG: 20,
    LANE_RESET_NO_WALLS_CM: 60,
    // Stale sensor
    STALE_MS: 500,
    // Median filter
    MEDIAN_WINDOW: 5,
    // Spin detection
    SPIN_OMEGA_THRESHOLD: 30,    // deg/s
    STOPPED_PWM_THRESHOLD: 5,    // consider stopped below this
});

// ---- UTILITY ---------------------------------------------------------------
const clamp  = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
const finite = (v) => typeof v === 'number' && Number.isFinite(v);
const DEG    = Math.PI / 180;
const RAD    = 180 / Math.PI;
const mod360 = (a) => ((a % 360) + 360) % 360;

function snapHeading(deg) {
    const a = mod360(deg);
    if (a < 45 || a >= 315) return { axis: 0, name: 'NORTH' };
    if (a < 135) return { axis: 90, name: 'EAST' };
    if (a < 225) return { axis: 180, name: 'SOUTH' };
    return { axis: 270, name: 'WEST' };
}

function angleDiff(a, b) {
    let d = mod360(a) - mod360(b);
    if (d > 180) d -= 360;
    if (d < -180) d += 360;
    return d;
}

// Sliding median: returns a fresh sorted copy middle value.
function median5(arr) {
    if (!arr || arr.length === 0) return 0;
    const sorted = arr.slice().sort((a, b) => a - b);
    return sorted[Math.floor(sorted.length / 2)];
}

// ---- SENSOR PACKET PARSER --------------------------------------------------
const SENSOR_HEADER = 0x5A;
const SENSOR_PKT_LEN = 9;

function parseSensorPacket(buf) {
    if (!buf || buf.length !== SENSOR_PKT_LEN) return null;
    if (buf[0] !== SENSOR_HEADER) return null;
    let xor = 0;
    for (let i = 0; i < SENSOR_PKT_LEN - 1; i++) xor ^= buf[i];
    if (xor !== buf[SENSOR_PKT_LEN - 1]) return null;
    return {
        seq: buf[1],
        leftUs:  buf[2] | (buf[3] << 8),
        frontUs: buf[4] | (buf[5] << 8),
        rightUs: buf[6] | (buf[7] << 8),
    };
}

function usToCm(us, cfg) {
    if (us === 0) return null;    // no echo
    const cm = us * cfg.SPEED_OF_SOUND_CM_PER_US / 2;
    // Rule 1b: A filtered reading below minimum range is NOT ignored. Clamp to 2.5 cm (blocked).
    if (cm < cfg.MIN_DIST_CM) return cfg.MIN_DIST_CM;
    if (cm > cfg.MAX_DIST_CM) return null;
    return cm;
}

// ---- MAZE STATE ------------------------------------------------------------
function createMazeState(cfg) {
    cfg = cfg || MAZE_CONFIG;
    const half = Math.floor(cfg.GRID_SIZE_CELLS / 2);
    return {
        cfg: cfg,
        // Calibration (persisted)
        vFull:     cfg.V_FULL_DEFAULT,
        spinDegS:  cfg.SPIN_DEG_S_DEFAULT,
        // Pose: x east, y north, heading degrees CW from north
        x: 0, y: 0, heading: 0,
        // Grid origin: cell (half, half) is world (0, 0)
        gridOriginCellX: half,
        gridOriginCellY: half,
        grid: new Float32Array(cfg.GRID_SIZE_CELLS * cfg.GRID_SIZE_CELLS),   // log-odds
        dirtyGrid: new Map(),    // cellIndex -> logOdd value (for incremental browser updates)
        fullSnapshotNeeded: true,
        // Graph
        nodes: [],    // { id, x, y, isJunction, isDeadEnd, turnDir, turnArrowHeading, explored }
        edges: [],    // { from, to }
        lastNodeId: -1,
        nextNodeId: 0,
        distSinceNode: 0,
        // Median filter buffers
        filterL: [], filterF: [], filterR: [],
        // Filtered readings (cm, null = invalid/open)
        distL: null, distF: null, distR: null,
        // Raw last µs
        rawL: 0, rawF: 0, rawR: 0,
        // Close reading memory timestamps (ms)
        lastCloseL: 0, lastCloseF: 0, lastCloseR: 0,
        // Sensor freshness
        lastSensorAt: 0,
        frontBlocked: false,
        // Motor state (from the server, post-ramp/deadband)
        pwmL: 0, pwmR: 0,
        // Odometry accumulators
        lastStepAt: 0,
        totalDist: 0,
        // V_FULL auto-tuning accumulators
        vfRefDist: 0,
        vfRefFront: null,
        vfDeadDist: 0,
        // Heading correction accumulators
        hcRefDist: 0,
        hcRefDiffLR: null,
        hcRefHeading: null,
        // Lane correction
        laneCentreSet: false,
        laneCentreCoord: 0,     // perpendicular-axis coordinate of the lane centre
        laneCentreAxis: 0,      // 0 = X lane (heading ~N/S), 90 = Y lane (heading ~E/W)
        laneDistWithoutWalls: 0,
        // Side openings
        openings: [],           // { x, y, side, entered, headingAtDetect }
        prevOpenL: false,
        prevOpenR: false,
        recentOpenL: null,      // { startX, startY, startDist, heading, active, endDist }
        recentOpenR: null,
        // Spin detection & turn tracking
        spinBaselineHeading: null,
        lastTurnTime: 0,
        lastTurnDir: null,
        distAtLastTurn: 0,
        // Spin calibration
        spinCalibActive: false,
        spinCalibAccum: 0,      // integral of |r-l|/510 * dt
        spinCalibStartAt: 0,
        // Turn log
        turnLog: [],
        prevSnapName: null,
        wasReversing: false,
        inDeadEnd: false,
        // Trail (for rendering)
        trail: [],              // { x, y }
        trailDist: 0,
        // Timer
        timerStartAt: 0,
        timerRunning: false,
        sessionStartAt: Date.now(),
        lastTickNow: Date.now(),
        // Paused
        mapActive: true,
    };
}

// ---- ODOMETRY STEP (call every server tick) ---------------------------------
function odomStep(state, dt) {
    if (dt <= 0 || dt > 0.2) return;
    const cfg = state.cfg;
    const l = state.pwmL, r = state.pwmR;

    // Speed (cm/s): v = V_FULL * (l+r) / 510
    // Omega (deg/s): ω = SPIN_DEG_S * (r-l) / 510
    const v = state.vFull * (l + r) / 510;
    const omega = state.spinDegS * (r - l) / 510;

    // Update heading
    state.heading = mod360(state.heading + omega * dt);

    // Update position
    const headRad = state.heading * DEG;
    const dx = v * Math.sin(headRad) * dt;
    const dy = v * Math.cos(headRad) * dt;
    state.x += dx;
    state.y += dy;

    const dist = Math.abs(v * dt);
    state.totalDist += dist;
    state.distSinceNode += dist;
    state.trailDist += dist;
    state.vfDeadDist += dist * Math.sign(v);
    state.laneDistWithoutWalls += dist;

    // Trail breadcrumb
    if (state.trailDist >= 1.0) {
        state.trail.push({ x: state.x, y: state.y });
        state.trailDist = 0;
    }

    // Spin calibration accumulator
    if (state.spinCalibActive) {
        state.spinCalibAccum += (Math.abs(r - l) / 510) * dt;
    }

    return { v, omega, dist };
}

// ---- SENSOR UPDATE (call when a new sensor packet arrives) -----------------
function sensorUpdate(state, pkt, now) {
    const cfg = state.cfg;
    state.rawL = pkt.leftUs;
    state.rawF = pkt.frontUs;
    state.rawR = pkt.rightUs;
    state.lastSensorAt = now;

    // Convert µs -> cm (clamps < 2.5 cm to 2.5 cm)
    const rawLcm = usToCm(pkt.leftUs, cfg);
    const rawFcm = usToCm(pkt.frontUs, cfg);
    const rawRcm = usToCm(pkt.rightUs, cfg);

    // Rule 1b: Close-range rules (apply to all 3 sensors)
    // 1) A filtered reading below min range is NOT ignored -> clamp to 2.5 cm (handled by usToCm).
    // 2) A "no echo" right after a close valid reading (previous median under 20 cm)
    //    is treated as still blocked/very close for up to 500 ms.
    // 3) Only a no-echo after far readings means open space.
    function resolveReading(rawCm, key) {
        if (rawCm !== null) {
            if (rawCm < cfg.CLOSE_THRESHOLD_CM) {
                state[key] = now;
            }
            return rawCm;
        }
        // No echo (us === 0 or > MAX_DIST_CM)
        if (state[key] && (now - state[key] <= cfg.CLOSE_MEMORY_MS)) {
            // Memory: still blocked / very close
            return cfg.MIN_DIST_CM;
        }
        // True open space
        return cfg.MAX_DIST_CM + 1;
    }

    const valL = resolveReading(rawLcm, 'lastCloseL');
    const valF = resolveReading(rawFcm, 'lastCloseF');
    const valR = resolveReading(rawRcm, 'lastCloseR');

    pushMedian(state.filterL, valL, cfg.MEDIAN_WINDOW);
    pushMedian(state.filterF, valF, cfg.MEDIAN_WINDOW);
    pushMedian(state.filterR, valR, cfg.MEDIAN_WINDOW);

    const ml = median5(state.filterL);
    const mf = median5(state.filterF);
    const mr = median5(state.filterR);

    state.distL = (ml <= cfg.MAX_DIST_CM) ? Math.max(cfg.MIN_DIST_CM, ml) : null;
    state.distF = (mf <= cfg.MAX_DIST_CM) ? Math.max(cfg.MIN_DIST_CM, mf) : null;
    state.distR = (mr <= cfg.MAX_DIST_CM) ? Math.max(cfg.MIN_DIST_CM, mr) : null;

    if (state.distL !== null && state.distL < cfg.CLOSE_THRESHOLD_CM) state.lastCloseL = now;
    if (state.distF !== null && state.distF < cfg.CLOSE_THRESHOLD_CM) state.lastCloseF = now;
    if (state.distR !== null && state.distR < cfg.CLOSE_THRESHOLD_CM) state.lastCloseR = now;

    // Rule 1c: FRONT BLOCKED flag whenever filtered front reading is under 12 cm
    state.frontBlocked = state.distF !== null && state.distF < cfg.FRONT_BLOCKED_CM;
}

function pushMedian(arr, value, window) {
    arr.push(value);
    if (arr.length > window) arr.shift();
}

// ---- V_FULL AUTO-CALIBRATION -----------------------------------------------
function vFullAutoTune(state, odom) {
    const cfg = state.cfg;
    if (!state.distF) { resetVfRef(state); return; }
    if (Math.abs(odom.omega) > cfg.SPIN_OMEGA_THRESHOLD) { resetVfRef(state); return; }
    if (state.distF < cfg.VFULL_FRONT_MIN_CM || state.distF > cfg.VFULL_FRONT_MAX_CM) { resetVfRef(state); return; }

    if (state.vfRefFront === null) {
        state.vfRefFront = state.distF;
        state.vfRefDist = 0;
        state.vfDeadDist = 0;
        return;
    }

    state.vfRefDist += Math.abs(odom.dist);
    if (state.vfRefDist < cfg.VFULL_WINDOW_CM) return;

    const deltaFront = state.vfRefFront - state.distF;
    const deadDist = state.vfDeadDist;
    if (Math.abs(deltaFront) < 1 || Math.abs(deadDist) < 1) { resetVfRef(state); return; }

    const ratio = deltaFront / deadDist;
    if (ratio < cfg.VFULL_RATIO_MIN || ratio > cfg.VFULL_RATIO_MAX) { resetVfRef(state); return; }

    state.vFull = clamp(state.vFull * (1 + cfg.VFULL_LEARN_RATE * (ratio - 1)),
                        cfg.V_FULL_MIN, cfg.V_FULL_MAX);
    resetVfRef(state);
}

function resetVfRef(state) {
    state.vfRefFront = null;
    state.vfRefDist = 0;
    state.vfDeadDist = 0;
}

// ---- HEADING CORRECTION (no gyro) -------------------------------------------
function headingCorrection(state, odom) {
    const cfg = state.cfg;
    if (!state.distL || !state.distR) { resetHcRef(state); return; }
    if (state.distL > cfg.BOTH_WALLS_CM || state.distR > cfg.BOTH_WALLS_CM) { resetHcRef(state); return; }
    if (Math.abs(odom.omega) > cfg.SPIN_OMEGA_THRESHOLD) { resetHcRef(state); return; }
    if (Math.abs(odom.v) < 1) { resetHcRef(state); return; }

    const diffLR = state.distL - state.distR;

    if (state.hcRefDiffLR === null) {
        state.hcRefDiffLR = diffLR;
        state.hcRefDist = 0;
        state.hcRefHeading = state.heading;
        return;
    }

    state.hcRefDist += Math.abs(odom.dist);
    if (state.hcRefDist < cfg.HEADING_CORR_DIST_CM) return;

    const slope = (diffLR - state.hcRefDiffLR) / state.hcRefDist;
    const tiltDeg = (slope / 2) * RAD;

    const snap = snapHeading(state.heading);
    const targetHeading = mod360(snap.axis + tiltDeg);
    const correction = angleDiff(targetHeading, state.heading);
    const limited = clamp(correction, -cfg.HEADING_CORR_MAX_DEG, cfg.HEADING_CORR_MAX_DEG);

    state.heading = mod360(state.heading + limited);
    resetHcRef(state);
}

function resetHcRef(state) {
    state.hcRefDiffLR = null;
    state.hcRefDist = 0;
    state.hcRefHeading = null;
}

// ---- SIDEWAYS POSITION CORRECTION -------------------------------------------
function laneCorrection(state, odom) {
    const cfg = state.cfg;
    if (Math.abs(odom.omega) > cfg.LANE_RESET_SPIN_DEG) {
        state.laneCentreSet = false;
        state.laneDistWithoutWalls = 0;
        return;
    }

    if (!state.distL || !state.distR) {
        if (state.laneDistWithoutWalls > cfg.LANE_RESET_NO_WALLS_CM) {
            state.laneCentreSet = false;
        }
        return;
    }
    state.laneDistWithoutWalls = 0;

    const snap = snapHeading(state.heading);
    const nearAxis = Math.abs(angleDiff(state.heading, snap.axis)) < 20;
    if (!nearAxis) return;

    const sensorOffset = (cfg.SENSOR_LEFT.x + cfg.SENSOR_RIGHT.x) / 2;
    const lateralShift = (state.distL - state.distR) / 2 + sensorOffset;

    let perpCoord = 0;
    let perpAxis = 0;
    let perpOffset = 0;

    if (snap.name === 'NORTH') {
        perpCoord = state.x; perpAxis = 0; perpOffset = lateralShift;
    } else if (snap.name === 'SOUTH') {
        perpCoord = state.x; perpAxis = 0; perpOffset = -lateralShift;
    } else if (snap.name === 'EAST') {
        perpCoord = state.y; perpAxis = 90; perpOffset = -lateralShift;
    } else if (snap.name === 'WEST') {
        perpCoord = state.y; perpAxis = 90; perpOffset = lateralShift;
    }

    if (!state.laneCentreSet || state.laneCentreAxis !== perpAxis) {
        state.laneCentreCoord = perpCoord - perpOffset;
        state.laneCentreAxis = perpAxis;
        state.laneCentreSet = true;
    }

    const corrected = state.laneCentreCoord + perpOffset;
    if (perpAxis === 0) {
        state.x = corrected;
    } else {
        state.y = corrected;
    }
}

// ---- OCCUPANCY GRID ---------------------------------------------------------
function worldToGrid(state, wx, wy) {
    const cfg = state.cfg;
    const cx = Math.round(wx / cfg.GRID_CELL_CM) + state.gridOriginCellX;
    const cy = Math.round(wy / cfg.GRID_CELL_CM) + state.gridOriginCellY;
    return { cx, cy };
}

function gridToWorld(state, cx, cy) {
    const cfg = state.cfg;
    return {
        wx: (cx - state.gridOriginCellX) * cfg.GRID_CELL_CM,
        wy: (cy - state.gridOriginCellY) * cfg.GRID_CELL_CM,
    };
}

function gridIndex(state, cx, cy) {
    const n = state.cfg.GRID_SIZE_CELLS;
    if (cx < 0 || cy < 0 || cx >= n || cy >= n) return -1;
    return cy * n + cx;
}

function updateGrid(state, odom) {
    const cfg = state.cfg;
    const headRad = state.heading * DEG;

    // Rule 1c: The grid must get an occupied cell at that distance on every update,
    // so the wall is drawn and does not fade (even when stopped).
    if (state.frontBlocked && state.distF !== null) {
        const fsx = state.x + cfg.SENSOR_FRONT.x * Math.cos(headRad) + cfg.SENSOR_FRONT.y * Math.sin(headRad);
        const fsy = state.y - cfg.SENSOR_FRONT.x * Math.sin(headRad) + cfg.SENSOR_FRONT.y * Math.cos(headRad);
        const wx = fsx + Math.sin(headRad) * state.distF;
        const wy = fsy + Math.cos(headRad) * state.distF;
        const g = worldToGrid(state, wx, wy);
        const idx = gridIndex(state, g.cx, g.cy);
        if (idx >= 0) {
            state.grid[idx] = clamp(state.grid[idx] + cfg.LOG_ODD_OCC, -cfg.LOG_ODD_CLAMP, cfg.LOG_ODD_CLAMP);
            state.dirtyGrid.set(idx, state.grid[idx]);
        }
    }

    // Freeze general map raycast while stopped (so noise doesn't build up)
    if (Math.abs(state.pwmL) < cfg.STOPPED_PWM_THRESHOLD &&
        Math.abs(state.pwmR) < cfg.STOPPED_PWM_THRESHOLD) return;

    const spinning = odom && Math.abs(odom.omega) > cfg.SPIN_OMEGA_THRESHOLD;
    const weight = spinning ? cfg.LOG_ODD_SPIN_SCALE : 1.0;

    const sensors = [
        { dist: state.distL, rawUs: state.rawL, mount: cfg.SENSOR_LEFT },
        { dist: state.distF, rawUs: state.rawF, mount: cfg.SENSOR_FRONT },
        { dist: state.distR, rawUs: state.rawR, mount: cfg.SENSOR_RIGHT },
    ];

    for (const s of sensors) {
        const sensorWorldX = state.x + s.mount.x * Math.cos(headRad) + s.mount.y * Math.sin(headRad);
        const sensorWorldY = state.y - s.mount.x * Math.sin(headRad) + s.mount.y * Math.cos(headRad);

        const beamAngle = (state.heading + s.mount.angle_deg) * DEG;
        const beamDirX = Math.sin(beamAngle);
        const beamDirY = Math.cos(beamAngle);

        let maxDist;
        let hitWall = false;
        if (s.dist !== null) {
            maxDist = s.dist;
            hitWall = true;
        } else {
            maxDist = cfg.NO_ECHO_FREE_CM;
            hitWall = false;
        }

        const stepCm = cfg.GRID_CELL_CM;
        const steps = Math.ceil(maxDist / stepCm);
        for (let i = 0; i <= steps; i++) {
            const d = Math.min(i * stepCm, maxDist);
            const wx = sensorWorldX + beamDirX * d;
            const wy = sensorWorldY + beamDirY * d;
            const g = worldToGrid(state, wx, wy);
            const idx = gridIndex(state, g.cx, g.cy);
            if (idx < 0) continue;

            let delta;
            if (i === steps && hitWall) {
                delta = cfg.LOG_ODD_OCC * weight;
            } else {
                delta = (hitWall ? cfg.LOG_ODD_FREE : cfg.LOG_ODD_NO_ECHO_FREE) * weight;
            }

            const oldVal = state.grid[idx];
            state.grid[idx] = clamp(oldVal + delta, -cfg.LOG_ODD_CLAMP, cfg.LOG_ODD_CLAMP);

            if (state.grid[idx] !== oldVal) {
                state.dirtyGrid.set(idx, state.grid[idx]);
            }
        }
    }
}

// ---- GRAPH ------------------------------------------------------------------
function graphUpdate(state, odom) {
    const cfg = state.cfg;
    const omega = odom ? Math.abs(odom.omega) : 0;
    const snap = snapHeading(state.heading);
    const nearAxis = Math.abs(angleDiff(state.heading, snap.axis)) < 25;
    const isDeadEndNow = nearAxis && omega < cfg.SPIN_OMEGA_THRESHOLD &&
                         state.distF !== null && state.distF < cfg.DEAD_END_FRONT_CM &&
                         state.distL !== null && state.distL < cfg.BOTH_WALLS_CM &&
                         state.distR !== null && state.distR < cfg.BOTH_WALLS_CM;

    // Check if near any existing node (prevents duplicates when backtracking)
    for (const n of state.nodes) {
        if (Math.hypot(state.x - n.x, state.y - n.y) < cfg.NODE_MERGE_CM) {
            if (state.lastNodeId >= 0 && state.lastNodeId !== n.id) {
                const exists = state.edges.some(e =>
                    (e.from === state.lastNodeId && e.to === n.id) ||
                    (e.from === n.id && e.to === state.lastNodeId));
                if (!exists) {
                    state.edges.push({ from: state.lastNodeId, to: n.id });
                }
            }
            state.lastNodeId = n.id;
            state.distSinceNode = 0;
            if (isDeadEndNow) {
                n.isDeadEnd = true;
                n.explored = true;
            }
            break;
        }
    }

    // Dead end detection
    if (isDeadEndNow) {
        if (!state.inDeadEnd) {
            state.inDeadEnd = true;
            let deadNodeId = state.lastNodeId;
            let merged = state.nodes.find(n => Math.hypot(state.x - n.x, state.y - n.y) < cfg.NODE_MERGE_CM);
            if (merged) {
                merged.isDeadEnd = true;
                merged.explored = true;
                deadNodeId = merged.id;
            } else {
                deadNodeId = state.nextNodeId++;
                state.nodes.push({
                    id: deadNodeId, x: state.x, y: state.y,
                    isJunction: false, isDeadEnd: true, explored: true
                });
                if (state.lastNodeId >= 0 && deadNodeId !== state.lastNodeId) {
                    state.edges.push({ from: state.lastNodeId, to: deadNodeId });
                }
                state.lastNodeId = deadNodeId;
            }
            addTurnLog(state, `DEAD END (node #${deadNodeId})`);
            state.distSinceNode = 0;
        }
    } else if (state.distF === null || state.distF > cfg.DEAD_END_FRONT_CM + 8) {
        state.inDeadEnd = false;
    }

    if (state.distSinceNode < cfg.NODE_SPACING_CM) return;

    // Check if an existing node is nearby (merge)
    let mergedId = -1;
    for (const n of state.nodes) {
        if (Math.hypot(state.x - n.x, state.y - n.y) < cfg.NODE_MERGE_CM) {
            mergedId = n.id;
            break;
        }
    }

    let nodeId;
    if (mergedId >= 0) {
        nodeId = mergedId;
    } else {
        nodeId = state.nextNodeId++;
        const isJunction = (state.distL === null || state.distL > cfg.OPENING_THRESHOLD_CM) ||
                           (state.distR === null || state.distR > cfg.OPENING_THRESHOLD_CM);
        state.nodes.push({
            id: nodeId, x: state.x, y: state.y,
            isJunction: isJunction, isDeadEnd: isDeadEndNow, explored: false
        });
    }

    if (state.lastNodeId >= 0 && nodeId !== state.lastNodeId) {
        const exists = state.edges.some(e =>
            (e.from === state.lastNodeId && e.to === nodeId) ||
            (e.from === nodeId && e.to === state.lastNodeId));
        if (!exists) {
            state.edges.push({ from: state.lastNodeId, to: nodeId });
        }
    }

    state.lastNodeId = nodeId;
    state.distSinceNode = 0;
}

// ---- SIDE OPENINGS ----------------------------------------------------------
function detectOpenings(state, odom) {
    const cfg = state.cfg;
    if (Math.abs(odom.v) < 1 || Math.abs(odom.omega) > cfg.SPIN_OMEGA_THRESHOLD) return;

    const openL = state.distL === null || state.distL > cfg.OPENING_THRESHOLD_CM;
    const openR = state.distR === null || state.distR > cfg.OPENING_THRESHOLD_CM;

    // Rising edges: start tracking opening travel
    if (openL && !state.prevOpenL) {
        state.recentOpenL = {
            startX: state.x,
            startY: state.y,
            startDist: state.totalDist,
            heading: state.heading,
            active: true,
            endDist: null
        };
        addOpening(state, 'left', odom);
    } else if (openL && state.recentOpenL) {
        state.recentOpenL.active = true;
    }

    if (openR && !state.prevOpenR) {
        state.recentOpenR = {
            startX: state.x,
            startY: state.y,
            startDist: state.totalDist,
            heading: state.heading,
            active: true,
            endDist: null
        };
        addOpening(state, 'right', odom);
    } else if (openR && state.recentOpenR) {
        state.recentOpenR.active = true;
    }

    // Falling edges with hysteresis
    if (!openL && state.prevOpenL) {
        if (state.distL !== null && state.distL < cfg.OPENING_THRESHOLD_CM - cfg.OPENING_HYSTERESIS_CM) {
            state.prevOpenL = false;
            if (state.recentOpenL) {
                state.recentOpenL.active = false;
                state.recentOpenL.endDist = state.totalDist;
            }
        }
    } else {
        state.prevOpenL = openL;
    }

    if (!openR && state.prevOpenR) {
        if (state.distR !== null && state.distR < cfg.OPENING_THRESHOLD_CM - cfg.OPENING_HYSTERESIS_CM) {
            state.prevOpenR = false;
            if (state.recentOpenR) {
                state.recentOpenR.active = false;
                state.recentOpenR.endDist = state.totalDist;
            }
        }
    } else {
        state.prevOpenR = openR;
    }

    // Check if any unexplored opening has been entered
    for (const op of state.openings) {
        if (op.entered) continue;
        const dx = state.x - op.x, dy = state.y - op.y;
        if (Math.hypot(dx, dy) < cfg.OPENING_ENTER_CM) {
            const headSnap = snapHeading(state.heading);
            const opSnap = snapHeading(op.headingAtDetect);
            if (headSnap.name !== opSnap.name) {
                op.entered = true;
            }
        }
    }
}

function addOpening(state, side, odom) {
    const cfg = state.cfg;
    for (const op of state.openings) {
        if (op.side !== side) continue;
        if (Math.hypot(state.x - op.x, state.y - op.y) < cfg.OPENING_DEDUP_CM) return;
    }

    state.openings.push({
        x: state.x, y: state.y,
        side: side, entered: false,
        headingAtDetect: state.heading,
    });
    addTurnLog(state, (side === 'left' ? 'LEFT' : 'RIGHT') + ' OPENING');
}

// ---- TURN LOG ---------------------------------------------------------------
function addTurnLog(state, text) {
    let elapsed = 0;
    const now = state.lastTickNow !== undefined ? state.lastTickNow : Date.now();
    if (state.timerRunning && state.timerStartAt > 0) {
        elapsed = Math.floor(Math.max(0, now - state.timerStartAt) / 1000);
    } else if (state.sessionStartAt !== undefined) {
        if (state.sessionStartAt === 0 || now < state.sessionStartAt) {
            elapsed = Math.floor(Math.max(0, now) / 1000);
        } else {
            elapsed = Math.floor(Math.max(0, now - state.sessionStartAt) / 1000);
        }
    }
    const mins = Math.floor(elapsed / 60);
    const secs = elapsed % 60;
    const time = String(mins) + ':' + String(secs).padStart(2, '0');
    state.turnLog.push({ time, text });
}

// ---- JUNCTION RE-ANCHORING & TURNS (PART 2) ----------------------------------
function reanchorToJunction(state, dir) {
    const cfg = state.cfg;
    const op = dir === 'left' ? state.recentOpenL : state.recentOpenR;
    let juncX = state.x;
    let juncY = state.y;
    const oldHeading = (op && op.heading !== undefined) ? op.heading : state.heading;

    // Rule: opening beside car OR ended within last 25 cm of travel
    let reanchored = false;
    if (op) {
        const isBeside = op.active;
        const endedWithin25 = !op.active && op.endDist !== null && (state.totalDist - op.endDist <= 25);
        if (isBeside || endedWithin25) {
            const oldHeadRad = oldHeading * DEG;
            juncX = op.startX + cfg.ADVANCE_CM * Math.sin(oldHeadRad);
            juncY = op.startY + cfg.ADVANCE_CM * Math.cos(oldHeadRad);
            state.x = juncX;
            state.y = juncY;
            reanchored = true;
        }
    }

    const newHeading = mod360(oldHeading + (dir === 'left' ? -90 : 90));
    state.heading = newHeading;
    state.prevSnapName = snapHeading(newHeading).name;
    state.laneCentreSet = false;

    // Place junction node at the junction centre
    let juncNode = null;
    for (const n of state.nodes) {
        if (Math.hypot(n.x - juncX, n.y - juncY) < cfg.NODE_MERGE_CM) {
            juncNode = n;
            break;
        }
    }

    if (!juncNode) {
        juncNode = {
            id: state.nextNodeId++,
            x: juncX,
            y: juncY,
            isJunction: true,
            isDeadEnd: false,
            turnDir: dir,
            turnArrowHeading: newHeading,
            explored: false
        };
        state.nodes.push(juncNode);
    } else {
        juncNode.isJunction = true;
        juncNode.turnDir = dir;
        juncNode.turnArrowHeading = newHeading;
    }

    if (state.lastNodeId >= 0 && state.lastNodeId !== juncNode.id) {
        const exists = state.edges.some(e =>
            (e.from === state.lastNodeId && e.to === juncNode.id) ||
            (e.from === juncNode.id && e.to === state.lastNodeId));
        if (!exists) {
            state.edges.push({ from: state.lastNodeId, to: juncNode.id });
        }
    }

    state.lastNodeId = juncNode.id;
    state.distSinceNode = 0;

    // Turn log entry: LEFT, RIGHT with node id
    addTurnLog(state, `${dir.toUpperCase()} (node #${juncNode.id})`);
    return juncNode;
}

function handleUturn(state) {
    const cfg = state.cfg;
    state.heading = mod360(state.heading + 180);
    state.prevSnapName = snapHeading(state.heading).name;
    state.laneCentreSet = false;

    // Do NOT create duplicate nodes: continue along existing nodes and edges
    let curNode = null;
    for (const n of state.nodes) {
        if (Math.hypot(n.x - state.x, n.y - state.y) < cfg.NODE_MERGE_CM) {
            curNode = n;
            break;
        }
    }
    if (!curNode && state.lastNodeId >= 0) {
        curNode = state.nodes.find(n => n.id === state.lastNodeId);
    }

    const nodeId = curNode ? curNode.id : (state.lastNodeId >= 0 ? state.lastNodeId : 0);
    if (state.inDeadEnd && curNode) {
        curNode.isDeadEnd = true;
        curNode.explored = true;
    }
    if (curNode) {
        state.lastNodeId = curNode.id;
    }
    state.distSinceNode = 0;

    // Turn log entry: U-TURN with node id
    addTurnLog(state, `U-TURN (node #${nodeId})`);
    return { heading: state.heading, nodeId };
}

function detectTurns(state, odom) {
    const cfg = state.cfg;
    const now = state.lastTickNow !== undefined ? state.lastTickNow : Date.now();
    const snap = snapHeading(state.heading);

    if (state.prevSnapName === null) {
        state.prevSnapName = snap.name;
        state.spinBaselineHeading = state.heading;
        return;
    }

    const spinning = odom && Math.abs(odom.omega) > cfg.SPIN_OMEGA_THRESHOLD;
    if (spinning) {
        if (state.spinBaselineHeading === null) state.spinBaselineHeading = state.heading;
        const spinAngle = angleDiff(state.heading, state.spinBaselineHeading);

        // U-turn detection: spin of ~180°
        if (Math.abs(spinAngle) >= 150 && Math.abs(spinAngle) <= 210) {
            handleUturn(state);
            state.spinBaselineHeading = state.heading;
            state.prevSnapName = snap.name;
            return;
        }

        // A/D spin of at least 45°
        if (Math.abs(spinAngle) >= 45) {
            const dir = spinAngle > 0 ? 'right' : 'left';
            // Check: two same-direction turns within 2 seconds with no travel between them
            const isDoubleTurn = (state.lastTurnDir === dir) &&
                                 (now - state.lastTurnTime < 2000) &&
                                 (state.totalDist - state.distAtLastTurn < 5);

            if (isDoubleTurn) {
                handleUturn(state);
            } else {
                reanchorToJunction(state, dir);
            }

            state.lastTurnDir = dir;
            state.lastTurnTime = now;
            state.distAtLastTurn = state.totalDist;
            state.spinBaselineHeading = state.heading;
            state.prevSnapName = snap.name;
        }
    } else {
        state.spinBaselineHeading = state.heading;
        state.prevSnapName = snap.name;
    }

    // Detect reversing
    const reversing = (state.pwmL + state.pwmR) < -10;
    if (reversing && !state.wasReversing) {
        addTurnLog(state, 'BACK');
    }
    state.wasReversing = reversing;
}

// ---- SPIN CALIBRATION -------------------------------------------------------
function toggleSpinCalib(state) {
    if (!state.spinCalibActive) {
        state.spinCalibActive = true;
        state.spinCalibAccum = 0;
        state.spinCalibStartAt = Date.now();
        addTurnLog(state, 'SPIN CALIB START');
        return { started: true };
    } else {
        state.spinCalibActive = false;
        if (state.spinCalibAccum > 0.01) {
            state.spinDegS = 360 / state.spinCalibAccum;
            state.spinDegS = clamp(state.spinDegS, 30, 1000);
            addTurnLog(state, 'SPIN CALIB: ' + state.spinDegS.toFixed(1) + ' deg/s');
            return { started: false, spinDegS: state.spinDegS };
        }
        addTurnLog(state, 'SPIN CALIB: too short, ignored');
        return { started: false, spinDegS: null };
    }
}

// ---- MAIN TICK (called from server every TICK_MS) ---------------------------
function mazeTick(state, dt, now) {
    state.lastTickNow = now;
    if (!state.mapActive) return { odom: { v: 0, omega: 0, dist: 0 } };

    const odom = odomStep(state, dt) || { v: 0, omega: 0, dist: 0 };

    const sensorFresh = (now - state.lastSensorAt) < state.cfg.STALE_MS;
    if (sensorFresh) {
        vFullAutoTune(state, odom);
        headingCorrection(state, odom);
        laneCorrection(state, odom);
        updateGrid(state, odom);
        graphUpdate(state, odom);
        detectOpenings(state, odom);
        detectTurns(state, odom);
    }

    return { odom };
}

// ---- RESET ------------------------------------------------------------------
function resetMaze(state) {
    const cfg = state.cfg;
    const fresh = createMazeState(cfg);
    fresh.vFull = state.vFull;
    fresh.spinDegS = state.spinDegS;
    fresh.timerStartAt = state.timerStartAt;
    fresh.timerRunning = state.timerRunning;
    fresh.sessionStartAt = state.sessionStartAt;
    fresh.lastTickNow = state.lastTickNow;
    return fresh;
}

// ---- SERIALIZATION FOR BROWSER ----------------------------------------------
function getGridSnapshot(state) {
    const cells = [];
    const n = state.cfg.GRID_SIZE_CELLS;
    for (let i = 0; i < state.grid.length; i++) {
        if (state.grid[i] !== 0) {
            const cx = i % n, cy = Math.floor(i / n);
            const w = gridToWorld(state, cx, cy);
            cells.push({ x: w.wx, y: w.wy, v: Math.round(state.grid[i] * 100) / 100 });
        }
    }
    return cells;
}

function getDirtyGridCells(state) {
    const cells = [];
    const n = state.cfg.GRID_SIZE_CELLS;
    for (const [idx, val] of state.dirtyGrid) {
        const cx = idx % n, cy = Math.floor(idx / n);
        const w = gridToWorld(state, cx, cy);
        cells.push({ x: w.wx, y: w.wy, v: Math.round(val * 100) / 100 });
    }
    state.dirtyGrid.clear();
    return cells;
}

function getTelemetry(state, now) {
    const snap = snapHeading(state.heading);
    const stale = (now - state.lastSensorAt) > state.cfg.STALE_MS;
    const sensorAge = now - state.lastSensorAt;
    return {
        x: Math.round(state.x * 10) / 10,
        y: Math.round(state.y * 10) / 10,
        heading: Math.round(state.heading * 10) / 10,
        headingName: snap.name,
        distL: state.distL !== null ? Math.round(state.distL * 10) / 10 : null,
        distF: state.distF !== null ? Math.round(state.distF * 10) / 10 : null,
        distR: state.distR !== null ? Math.round(state.distR * 10) / 10 : null,
        rawL: state.rawL,
        rawF: state.rawF,
        rawR: state.rawR,
        frontBlocked: !!state.frontBlocked,
        pwmL: state.pwmL,
        pwmR: state.pwmR,
        vFull: Math.round(state.vFull * 10) / 10,
        spinDegS: Math.round(state.spinDegS * 10) / 10,
        sensorStale: stale,
        sensorAge: sensorAge,
        mapActive: state.mapActive,
        spinCalibActive: state.spinCalibActive,
        trail: state.trail,
        nodes: state.nodes,
        edges: state.edges,
        openings: state.openings,
        turnLog: state.turnLog,
        totalDist: Math.round(state.totalDist),
    };
}

// ---- MAZE CALIBRATION PERSISTENCE ------------------------------------------
function getMazeCalib(state) {
    return { vFull: state.vFull, spinDegS: state.spinDegS };
}

function applyMazeCalib(state, cal) {
    if (cal && finite(cal.vFull)) state.vFull = clamp(cal.vFull, state.cfg.V_FULL_MIN, state.cfg.V_FULL_MAX);
    if (cal && finite(cal.spinDegS)) state.spinDegS = clamp(cal.spinDegS, 30, 1000);
}

// ---- SENSOR BEAM ENDPOINTS (for UI) ----------------------------------------
function getSensorBeams(state) {
    const cfg = state.cfg;
    const headRad = state.heading * DEG;
    const beams = [];
    const sensors = [
        { dist: state.distL, mount: cfg.SENSOR_LEFT, name: 'L' },
        { dist: state.distF, mount: cfg.SENSOR_FRONT, name: 'F' },
        { dist: state.distR, mount: cfg.SENSOR_RIGHT, name: 'R' },
    ];
    for (const s of sensors) {
        const sx = state.x + s.mount.x * Math.cos(headRad) + s.mount.y * Math.sin(headRad);
        const sy = state.y - s.mount.x * Math.sin(headRad) + s.mount.y * Math.cos(headRad);
        const d = s.dist !== null ? s.dist : cfg.NO_ECHO_FREE_CM;
        const beamAngle = (state.heading + s.mount.angle_deg) * DEG;
        const ex = sx + Math.sin(beamAngle) * d;
        const ey = sy + Math.cos(beamAngle) * d;
        beams.push({ name: s.name, sx, sy, ex, ey, hit: s.dist !== null, dist: s.dist });
    }
    return beams;
}

// ---- CAR RECTANGLE (for UI) -------------------------------------------------
function getCarRect(state) {
    const cfg = state.cfg;
    const headRad = state.heading * DEG;
    const hw = cfg.CAR_WIDTH_CM / 2;
    const hl = cfg.CAR_LENGTH_CM / 2;
    const corners = [
        { cx: -hw, cy:  hl },   // front-left
        { cx:  hw, cy:  hl },   // front-right
        { cx:  hw, cy: -hl },   // back-right
        { cx: -hw, cy: -hl },   // back-left
    ];
    return corners.map(c => ({
        x: state.x + c.cx * Math.cos(headRad) + c.cy * Math.sin(headRad),
        y: state.y - c.cx * Math.sin(headRad) + c.cy * Math.cos(headRad),
    }));
}

// ---- MANUAL TURN / CARDINAL SNAP (driver dedicated keys) -------------------
function manualTurn(state, action) {
    if (action === 'left') {
        reanchorToJunction(state, 'left');
    } else if (action === 'right') {
        reanchorToJunction(state, 'right');
    } else if (action === 'uturn' || action === 'back') {
        handleUturn(state);
    } else {
        let newAxis = 0;
        let label = '';
        if (action === 'north') { newAxis = 0; label = 'FACING NORTH'; }
        else if (action === 'east') { newAxis = 90; label = 'FACING EAST'; }
        else if (action === 'south') { newAxis = 180; label = 'FACING SOUTH'; }
        else if (action === 'west') { newAxis = 270; label = 'FACING WEST'; }

        state.heading = newAxis;
        state.prevSnapName = snapHeading(newAxis).name;
        state.laneCentreSet = false;

        let merged = state.nodes.find(n => Math.hypot(state.x - n.x, state.y - n.y) < state.cfg.NODE_MERGE_CM);
        let nodeId = merged ? merged.id : state.nextNodeId++;
        if (!merged) {
            state.nodes.push({ id: nodeId, x: state.x, y: state.y, isJunction: false, isDeadEnd: false });
        }
        if (state.lastNodeId >= 0 && nodeId !== state.lastNodeId) {
            const exists = state.edges.some(e =>
                (e.from === state.lastNodeId && e.to === nodeId) ||
                (e.from === nodeId && e.to === state.lastNodeId));
            if (!exists) state.edges.push({ from: state.lastNodeId, to: nodeId });
        }
        state.lastNodeId = nodeId;
        state.distSinceNode = 0;
        if (label) addTurnLog(state, label);
    }

    return { heading: state.heading, headingName: state.prevSnapName };
}

module.exports = {
    MAZE_CONFIG,
    createMazeState,
    parseSensorPacket,
    sensorUpdate,
    odomStep,
    mazeTick,
    graphUpdate,
    resetMaze,
    toggleSpinCalib,
    manualTurn,
    reanchorToJunction,
    handleUturn,
    getGridSnapshot,
    getDirtyGridCells,
    getTelemetry,
    getMazeCalib,
    applyMazeCalib,
    getSensorBeams,
    getCarRect,
    snapHeading,
    angleDiff,
    mod360,
    clamp,
    usToCm,
    median5,
    addTurnLog,
};
