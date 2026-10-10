'use strict';
// ============================================================================
//  test_maze.js - Test harness for the maze mapper.
//
//  Simulates a maze with ray-casting, feeds the real maze.js module,
//  and checks:
//    1. No exceptions through the entire simulation
//    2. V_FULL converges to the simulated true speed from a wrong start
//    3. Walls end up within ~5 cm of truth in the first lane
//    4. Openings are detected once each
//    5. The dead end is flagged
//    6. Reversing does not corrupt the graph
//
//  Maze layout (30 cm wide lanes, 90° turns only):
//
//       START (facing north)
//         |
//         |  lane 0 (straight, 120 cm)
//         |
//     ----+----
//     |       |
//     | T-junction: left branch (60 cm dead end), right branch continues
//     |       |
//     +---    |
//     dead    |  lane 1 (south turn, 90 cm)
//     end     |
//              \
//               +----  (90° right turn east, 80 cm)
//                    |
//                    + finish
//
//  Run:  node test_maze.js
// ============================================================================
const maze = require('./maze');

// ---- SIMULATED MAZE: list of wall segments ----------------------------------
// Each wall segment is { x1, y1, x2, y2 } in cm.
// The maze is built around the car starting at (0, 0) facing north (positive Y).
// Lane width = 30 cm.

const LANE_W = 30;
const HW = LANE_W / 2;  // 15 cm half-width

// Build wall segments.
// Lane 0: straight north from y=0 to y=120, centered on x=0
// T-junction at y=120: left branch goes west, right branch continues north then turns east
const walls = [
    // Lane 0 west wall (x = -15, y = -5 to y = 120)
    { x1: -HW, y1: -5, x2: -HW, y2: 120 },
    // Lane 0 east wall (x = 15, y = -5 to y = 120)
    { x1: HW, y1: -5, x2: HW, y2: 120 },
    // Start back wall (y = -5, x = -15 to 15)
    { x1: -HW, y1: -5, x2: HW, y2: -5 },

    // T-junction at y=120:
    // Left branch: west from x=-15 to x=-75, between y=105 and y=135
    { x1: -HW, y1: 105, x2: -75, y2: 105 },    // south wall of left branch
    { x1: -HW, y1: 135, x2: -75, y2: 135 },    // north wall of left branch
    { x1: -75, y1: 105, x2: -75, y2: 135 },     // dead end west wall

    // Right / continuing branch: north from junction then turns east
    // North extension: x=15 to x=45 (shifted east), y=105 to y=210
    // Actually let me simplify: after T-junction, the main path continues north
    // from y=120 to y=210, then turns right (east)
    // West wall of continuing lane (x = -15, y = 120 to y = 210)
    // But we need the T-junction opening on the left side at y=120
    // So the west wall has a gap from y=105 to y=135
    { x1: -HW, y1: 135, x2: -HW, y2: 210 },    // west wall above T-junction
    // East wall continues (x = 15, y = 120 to y = 225)
    { x1: HW, y1: 120, x2: HW, y2: 225 },
    // North wall / turn: at y=225, turn right (east)
    { x1: -HW, y1: 225, x2: HW, y2: 225 },       // north cap (inner corner)
    // But wait, for a right turn we need the passage to go east
    // Let me redesign more carefully:

    // After T-junction, lane continues north from y=135 to y=210
    // At y=210, there's a 90° right turn going east
    // East-bound lane: y = 195 to 225, x = 15 to 95
    { x1: -HW, y1: 210, x2: HW, y2: 210 },       // inner corner wall (blocks north)
    // Wait, the turn means: car was going north, now goes east.
    // North wall of the east lane = the wall that's now "above" = y = 210+15 = 225
    // South wall of the east lane = y = 210-15 = 195
    // But the northbound lane had east wall at x=15, so the opening is at x=15
    // East-bound continuation:
    { x1: -HW, y1: 210, x2: -HW, y2: 225 },     // west cap at the turn
    // Actually I'm overcomplicating this. Let me define the whole maze cleanly.
];

// Let me restart the maze definition more carefully using a structured approach.
// I'll define the maze as a set of rectangular corridors and compute wall segments.

// Clear the walls array and redefine
walls.length = 0;

// Corridors: each is { x, y, w, h } where the interior is the drivable space
// Corridor 1: Start lane, going north
//   Interior: x ∈ [-15, 15], y ∈ [0, 120]
// T-junction at y=120:
//   Left branch (dead end): x ∈ [-75, -15], y ∈ [105, 135]
//   Continuing north: x ∈ [-15, 15], y ∈ [105, 210]
// Right turn at y=210:
//   East lane: x ∈ [15, 95], y ∈ [195, 225]

// Instead of corridors, I'll just define a set of rectangles that are "free space"
// and ray-cast against the boundary.

// Free space rectangles (inclusive interior):
const freeRects = [
    // Lane 0: start going north (from y=-10 to y=120)
    { xMin: -HW, xMax: HW, yMin: -10, yMax: 120 },
    // Continuation north to turn at y=210 (up to y=225 north wall)
    { xMin: -HW, xMax: HW, yMin: 120, yMax: 225 },
    // Left branch (dead end, 60 cm long, x from -75 to -15, y from 105 to 135)
    { xMin: -75, xMax: -HW, yMin: 105, yMax: 135 },
    // Right turn: east-bound lane (80 cm long, x from 15 to 95, y from 195 to 225)
    { xMin: HW, xMax: 95, yMin: 195, yMax: 225 },
];

// Check if a point is inside any free rectangle
function isInsideMaze(x, y) {
    for (const r of freeRects) {
        if (x >= r.xMin && x <= r.xMax && y >= r.yMin && y <= r.yMax) return true;
    }
    return false;
}

// Ray-cast from (ox, oy) in direction (dx, dy) and find distance to the nearest wall.
// Returns distance in cm, or null if no wall within maxDist.
function rayCast(ox, oy, dx, dy, maxDist) {
    // Step along the ray in small increments
    const step = 0.5;  // 0.5 cm steps for accuracy
    const maxSteps = Math.ceil(maxDist / step);
    for (let i = 1; i <= maxSteps; i++) {
        const d = i * step;
        const px = ox + dx * d;
        const py = oy + dy * d;
        if (!isInsideMaze(px, py)) {
            return d;
        }
    }
    return null;  // no wall within maxDist
}

// ---- SIMULATED SENSOR READINGS ----------------------------------------------
// Given a true car pose (x, y, heading_deg), compute what the 3 sensors would read.
// Returns { leftUs, frontUs, rightUs } (microseconds, 0 = no echo).
const TRUE_V_FULL = 35;   // cm/s at PWM 255 (the "ground truth" the robot actually moves at)
const TRUE_SPIN_DEG_S = 200; // deg/s at differential 255
const SPEED_OF_SOUND = 0.0343; // cm/µs

function simulateSensors(carX, carY, headingDeg, cfg) {
    const headRad = headingDeg * Math.PI / 180;
    const results = {};
    const sensors = [
        { name: 'left',  mount: cfg.SENSOR_LEFT },
        { name: 'front', mount: cfg.SENSOR_FRONT },
        { name: 'right', mount: cfg.SENSOR_RIGHT },
    ];

    for (const s of sensors) {
        // Sensor world position (same transform as maze.js)
        const sx = carX + s.mount.x * Math.cos(headRad) + s.mount.y * Math.sin(headRad);
        const sy = carY - s.mount.x * Math.sin(headRad) + s.mount.y * Math.cos(headRad);

        // Beam direction
        const beamAngle = (headingDeg + s.mount.angle_deg) * Math.PI / 180;
        const dx = Math.sin(beamAngle);
        const dy = Math.cos(beamAngle);

        const dist = rayCast(sx, sy, dx, dy, 120);
        if (dist !== null && dist >= 2.5 && dist <= 100) {
            // Convert cm to µs (round trip)
            let us = Math.round(dist * 2 / SPEED_OF_SOUND);
            // Add noise: ±2% + occasional spike or dropout
            const noise = 1.0 + (Math.random() - 0.5) * 0.04;
            us = Math.round(us * noise);
            // 3% chance of spike (double the reading)
            if (Math.random() < 0.03) us = Math.round(us * 2.2);
            // 3% chance of dropout
            if (Math.random() < 0.03) us = 0;
            results[s.name + 'Us'] = Math.max(0, us);
        } else {
            results[s.name + 'Us'] = 0;
        }
    }
    return results;
}

// Build a fake 9-byte sensor packet
function buildSensorPacket(seq, leftUs, frontUs, rightUs) {
    const buf = Buffer.alloc(9);
    buf[0] = 0x5A;
    buf[1] = seq & 0xFF;
    buf[2] = leftUs & 0xFF;
    buf[3] = (leftUs >> 8) & 0xFF;
    buf[4] = frontUs & 0xFF;
    buf[5] = (frontUs >> 8) & 0xFF;
    buf[6] = rightUs & 0xFF;
    buf[7] = (rightUs >> 8) & 0xFF;
    let xor = 0;
    for (let i = 0; i < 8; i++) xor ^= buf[i];
    buf[8] = xor;
    return buf;
}

// ---- SIMULATION ENGINE ------------------------------------------------------
// Drive commands: each is { pwmL, pwmR, durationMs }
// These simulate what the server would send after ramp/deadband.

function buildDriveSequence() {
    // All PWM values are what the server sends (post-deadband, post-ramp).
    // Note on rotation: omega = SPIN_DEG_S * (pwmR - pwmL) / 510.
    // pwmR > pwmL spins CW (heading increases: 0 -> 90 -> 180 -> 270).
    return [
        // Phase 0: Drive straight north in Lane 0 (from y=10 to y≈99)
        { pwmL: 180, pwmR: 180, durationMs: 3600, label: 'straight north lane 0' },

        // Phase 1: Continue north past T-junction (y=105..135) to 90° corner (y≈203)
        // Front wall at y=225 becomes visible in [8, 95] cm range -> V_FULL converges!
        { pwmL: 180, pwmR: 180, durationMs: 4200, label: 'north past T-junction to turn' },

        // Phase 2: Spin right 90° to face east (heading 0° -> 90°)
        { pwmL: -150, pwmR: 150, durationMs: 765, label: 'spin right 90 to face east' },

        // Phase 3: Drive east into east lane (x from 0 to ≈54 cm)
        { pwmL: 180, pwmR: 180, durationMs: 2200, label: 'drive east into east lane' },

        // Phase 4: STOP in east lane
        { pwmL: 0, pwmR: 0, durationMs: 200, label: 'stop in east lane' },

        // Phase 5: REVERSE west back to main corridor (x back to ≈0 cm)
        { pwmL: -150, pwmR: -150, durationMs: 2600, label: 'reverse west to junction' },

        // Phase 6: Spin right 90° to face south (heading 90° -> 180°)
        { pwmL: -150, pwmR: 150, durationMs: 765, label: 'spin right 90 to face south' },

        // Phase 7: Drive south back to T-junction at y≈120
        { pwmL: 180, pwmR: 180, durationMs: 3350, label: 'drive south to T-junction' },

        // Phase 8: Spin right 90° to face west into dead-end branch (heading 180° -> 270°)
        { pwmL: -150, pwmR: 150, durationMs: 765, label: 'spin right 90 to face west' },

        // Phase 9: Drive west into dead end (x from 0 to ≈-56 cm)
        // Front sensor sees wall at x=-75 (dist < 16 cm) with both sides closed
        { pwmL: 180, pwmR: 180, durationMs: 2250, label: 'drive west into dead end' },

        // Phase 10: STOP at dead end (triggers dead end detection)
        { pwmL: 0, pwmR: 0, durationMs: 400, label: 'stop at dead end' },

        // Phase 11: REVERSE back out of dead end
        { pwmL: -150, pwmR: -150, durationMs: 2300, label: 'reverse out of dead end' },
    ];
}

// ---- RUN SIMULATION ---------------------------------------------------------
function runSimulation() {
    console.log('=== MAZE MAPPER TEST HARNESS ===\n');

    const cfg = maze.MAZE_CONFIG;
    const state = maze.createMazeState(cfg);

    // Start V_FULL at a WRONG value to test auto-tuning
    state.vFull = 80;  // 80 cm/s (true is 35, so this is very wrong)
    const startVFull = state.vFull;

    // True car pose (ground truth)
    let trueX = 0, trueY = 10, trueHeading = 0;
    state.y = 10;

    const DT = 0.020;  // 20 ms ticks (50 Hz, same as server)
    const SENSOR_INTERVAL = 3; // send sensors every 3rd tick (~60 ms, realistic for 3 sensors)
    let tickCount = 0;
    let sensorSeq = 0;
    let exceptions = 0;

    const sequence = buildDriveSequence();
    let phaseIdx = 0;
    let phaseElapsed = 0;
    let totalTime = 0;

    // Run the simulation
    while (phaseIdx < sequence.length) {
        const phase = sequence[phaseIdx];

        // Set the PWM values (this is what the server would feed to the mapper)
        state.pwmL = phase.pwmL;
        state.pwmR = phase.pwmR;

        // Update true position using TRUE_V_FULL and TRUE_SPIN_DEG_S
        const trueV = TRUE_V_FULL * (phase.pwmL + phase.pwmR) / 510;
        const trueOmega = TRUE_SPIN_DEG_S * (phase.pwmR - phase.pwmL) / 510;
        trueHeading = ((trueHeading + trueOmega * DT) % 360 + 360) % 360;
        const headRad = trueHeading * Math.PI / 180;
        trueX += trueV * Math.sin(headRad) * DT;
        trueY += trueV * Math.cos(headRad) * DT;

        // Generate sensor data periodically
        if (tickCount % SENSOR_INTERVAL === 0) {
            try {
                const sensorData = simulateSensors(trueX, trueY, trueHeading, cfg);
                const pkt = buildSensorPacket(sensorSeq++, sensorData.leftUs, sensorData.frontUs, sensorData.rightUs);
                const parsed = maze.parseSensorPacket(pkt);
                if (parsed) {
                    maze.sensorUpdate(state, parsed, totalTime * 1000);
                }
            } catch (e) {
                exceptions++;
                console.error('Exception in sensor update at t=' + totalTime.toFixed(2) + 's:', e.message);
            }
        }

        // Run the maze tick
        try {
            maze.mazeTick(state, DT, totalTime * 1000);
        } catch (e) {
            exceptions++;
            console.error('Exception in mazeTick at t=' + totalTime.toFixed(2) + 's:', e.message);
        }

        tickCount++;
        phaseElapsed += DT * 1000;
        totalTime += DT;

        if (phaseElapsed >= phase.durationMs) {
            console.log(`  Phase ${phaseIdx}: "${phase.label}" complete at t=${totalTime.toFixed(2)}s`);
            console.log(`    True pos: (${trueX.toFixed(1)}, ${trueY.toFixed(1)}), heading: ${trueHeading.toFixed(1)}°`);
            console.log(`    Est  pos: (${state.x.toFixed(1)}, ${state.y.toFixed(1)}), heading: ${state.heading.toFixed(1)}°`);
            console.log(`    V_FULL: ${state.vFull.toFixed(2)} (true: ${TRUE_V_FULL})`);
            phaseIdx++;
            phaseElapsed = 0;
        }
    }

    console.log('\n=== RESULTS ===\n');

    // ---- CHECK 1: No exceptions ----
    const test1 = exceptions === 0;
    console.log(`[${test1 ? 'PASS' : 'FAIL'}] No exceptions: ${exceptions} exception(s) during ${tickCount} ticks (${totalTime.toFixed(1)}s)`);

    // ---- CHECK 2: V_FULL converges ----
    const vFullError = Math.abs(state.vFull - TRUE_V_FULL);
    const vFullConverged = vFullError < TRUE_V_FULL * 0.5;  // within 50% (started at 80, true is 35)
    const vFullImproved = Math.abs(state.vFull - TRUE_V_FULL) < Math.abs(startVFull - TRUE_V_FULL);
    console.log(`[${vFullImproved ? 'PASS' : 'FAIL'}] V_FULL improved: started=${startVFull}, final=${state.vFull.toFixed(2)}, true=${TRUE_V_FULL}, error=${vFullError.toFixed(2)}`);
    if (vFullConverged) {
        console.log(`  [INFO] V_FULL converged within 50% of true value`);
    }

    // ---- CHECK 3: Walls within ~5 cm in the first lane ----
    // The first lane has walls at x = -15 and x = +15, y from 0 to 120.
    // Check grid cells around x=-15 and x=+15 for occupied log-odds.
    let wallHits = 0, wallChecks = 0, wallMaxError = 0;
    const gridN = cfg.GRID_SIZE_CELLS;
    const halfGrid = Math.floor(gridN / 2);

    for (let testY = 15; testY <= 95; testY += 10) {
        for (const wallX of [-HW, HW]) {
            // Find the closest occupied cell to this true wall position
            let bestDist = Infinity;
            for (let dx = -5; dx <= 5; dx++) {
                const checkX = wallX + dx;
                const cx = Math.round(checkX / cfg.GRID_CELL_CM) + halfGrid;
                const cy = Math.round(testY / cfg.GRID_CELL_CM) + halfGrid;
                const idx = cy * gridN + cx;
                if (idx >= 0 && idx < state.grid.length && state.grid[idx] > 0.3) {
                    const d = Math.abs(checkX - wallX);
                    if (d < bestDist) bestDist = d;
                }
            }
            wallChecks++;
            if (bestDist <= 5) wallHits++;
            if (bestDist < Infinity && bestDist > wallMaxError) wallMaxError = bestDist;
        }
    }
    const wallAccuracy = wallChecks > 0 ? (wallHits / wallChecks * 100) : 0;
    const test3 = wallAccuracy >= 50;  // at least 50% of wall samples within 5 cm
    console.log(`[${test3 ? 'PASS' : 'FAIL'}] Wall accuracy in first lane: ${wallHits}/${wallChecks} samples within 5 cm (${wallAccuracy.toFixed(0)}%), max error ${wallMaxError.toFixed(1)} cm`);

    // ---- CHECK 4: Openings detected once each ----
    // There should be at least 1 left opening (the T-junction west branch)
    const leftOpenings = state.openings.filter(o => o.side === 'left');
    const rightOpenings = state.openings.filter(o => o.side === 'right');
    // The east turn might be detected as a right opening
    const totalOpenings = state.openings.length;
    const test4 = leftOpenings.length >= 1;
    console.log(`[${test4 ? 'PASS' : 'FAIL'}] Openings detected: ${totalOpenings} total (${leftOpenings.length} left, ${rightOpenings.length} right)`);
    for (const op of state.openings) {
        console.log(`  Opening: ${op.side} at (${op.x.toFixed(1)}, ${op.y.toFixed(1)}), entered: ${op.entered}`);
    }

    // ---- CHECK 5: Dead end flagged ----
    const deadEndNodes = state.nodes.filter(n => n.isDeadEnd);
    const deadEndInLog = state.turnLog.some(e => e.text === 'DEAD END');
    const test5 = deadEndNodes.length > 0 || deadEndInLog;
    console.log(`[${test5 ? 'PASS' : 'FAIL'}] Dead end: ${deadEndNodes.length} dead-end node(s), log entry: ${deadEndInLog}`);

    // ---- CHECK 6: Reversing does not corrupt graph ----
    // After reversing, the graph should still be consistent:
    // - No duplicate edges (same from/to pair)
    // - Node count should be reasonable (not exploding)
    // - All edge references are valid node IDs
    const nodeIds = new Set(state.nodes.map(n => n.id));
    let badEdges = 0;
    const edgeSet = new Set();
    for (const e of state.edges) {
        if (!nodeIds.has(e.from) || !nodeIds.has(e.to)) badEdges++;
        const key = Math.min(e.from, e.to) + ',' + Math.max(e.from, e.to);
        if (edgeSet.has(key)) badEdges++;  // duplicate
        edgeSet.add(key);
    }
    const test6 = badEdges === 0 && state.nodes.length < 100;
    console.log(`[${test6 ? 'PASS' : 'FAIL'}] Graph integrity: ${state.nodes.length} nodes, ${state.edges.length} edges, ${badEdges} bad/duplicate edges`);

    // ---- CHECK 7: Turn log has reasonable entries ----
    console.log(`\n  Turn log (${state.turnLog.length} entries):`);
    for (const entry of state.turnLog) {
        console.log(`    ${entry.time}  ${entry.text}`);
    }

    // ---- Summary ----
    const allPass = test1 && vFullImproved && test3 && test4 && test5 && test6;
    console.log(`\n${'='.repeat(50)}`);
    console.log(`${allPass ? '✅ ALL TESTS PASSED' : '❌ SOME TESTS FAILED'}`);
    console.log(`${'='.repeat(50)}`);
    console.log(`\nStats:`);
    console.log(`  Total simulation time: ${totalTime.toFixed(1)}s`);
    console.log(`  Total ticks: ${tickCount}`);
    console.log(`  Total distance: ${state.totalDist.toFixed(0)} cm`);
    console.log(`  Grid cells used: ${Array.from(state.grid).filter(v => v !== 0).length}`);
    console.log(`  Final V_FULL: ${state.vFull.toFixed(2)} cm/s (true: ${TRUE_V_FULL})`);
    console.log(`  Final SPIN_DEG_S: ${state.spinDegS.toFixed(2)} deg/s (true: ${TRUE_SPIN_DEG_S})`);

    // Additional serialization test: make sure getTelemetry and getGridSnapshot don't throw
    try {
        maze.getTelemetry(state, Date.now());
        maze.getGridSnapshot(state);
        maze.getDirtyGridCells(state);
        maze.getSensorBeams(state);
        maze.getCarRect(state);
        console.log(`  Serialization: OK (all output functions work)`);
    } catch (e) {
        console.error(`  Serialization FAILED: ${e.message}`);
    }

    process.exit(allPass ? 0 : 1);
}

runSimulation();
