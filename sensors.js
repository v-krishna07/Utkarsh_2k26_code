'use strict';
// ============================================================================
//  sensors.js - Non-blocking HC-SR04 packet parsing, filtering & advice logic.
//
//  All calculations run on the laptop; the ESP32 performs no math.
//  Packet format (9 bytes, UDP):
//      [0] 0x5A            header
//      [1] seq             rolling sequence number (0..255)
//      [2] L_lo            left  echo microseconds, low  byte
//      [3] L_hi            left  echo microseconds, high byte  (0 = no echo)
//      [4] F_lo            front echo microseconds, low  byte
//      [5] F_hi            front echo microseconds, high byte
//      [6] R_lo            right echo microseconds, low  byte
//      [7] R_hi            right echo microseconds, high byte
//      [8] xor             XOR of bytes 0..7
// ============================================================================

const SENSOR_HEADER = 0x5A;
const SENSOR_PKT_LEN = 9;

const CONFIG = Object.freeze({
    SPEED_OF_SOUND_CM_PER_US: 0.0343,   // at ~20 °C
    MIN_DIST_CM: 2.5,
    MAX_DIST_CM: 100,
    CLOSE_THRESHOLD_CM: 20,             // readings under 20 cm trigger close-memory
    CLOSE_MEMORY_MS: 500,               // remember close reading for 500 ms on no-echo
    DEFAULT_FRONT_BLOCKED_CM: 12,
    DEFAULT_SIDE_OPEN_CM: 25,
    SIDE_WARN_CM: 5,
    HYSTERESIS_CM: 3,
    ADVICE_HOLD_COUNT: 3,
    FRONT_NO_SIGNAL_MS: 2000,
});

const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
const finite = (v) => typeof v === 'number' && Number.isFinite(v);

// Convert echo microseconds to centimetres
function usToCm(us, cfg = CONFIG) {
    if (!finite(us) || us <= 0) return null;
    const rawCm = (us * cfg.SPEED_OF_SOUND_CM_PER_US) / 2;
    if (rawCm < cfg.MIN_DIST_CM) return cfg.MIN_DIST_CM;
    if (rawCm > cfg.MAX_DIST_CM) return null;
    return rawCm;
}

// Parse and validate binary 9-byte sensor packet from the ESP32
function parseSensorPacket(buf) {
    if (!buf || buf.length !== SENSOR_PKT_LEN) return null;
    if (buf[0] !== SENSOR_HEADER) return null;

    let xor = 0;
    for (let i = 0; i < SENSOR_PKT_LEN - 1; i++) {
        xor ^= buf[i];
    }
    if (xor !== buf[SENSOR_PKT_LEN - 1]) return null;

    const seq = buf[1];
    const leftUs  = buf[2] | (buf[3] << 8);
    const frontUs = buf[4] | (buf[5] << 8);
    const rightUs = buf[6] | (buf[7] << 8);

    return { seq, leftUs, frontUs, rightUs };
}

// Compute effective N: integer 1..25, even values round up to next odd
function normalizeN(n) {
    if (!finite(n)) n = 5;
    n = Math.round(clamp(n, 1, 25));
    if (n % 2 === 0) n += 1;
    return clamp(n, 1, 25);
}

// Create fresh sensor state
function createSensorState(opts = {}) {
    const rawN = finite(opts.n) ? opts.n : 5;
    return {
        mode: opts.mode === 'raw' ? 'raw' : 'median',
        n: rawN,
        effectiveN: normalizeN(rawN),
        adviceEnabled: opts.adviceEnabled !== false,
        frontBlockedCm: finite(opts.frontBlockedCm) ? opts.frontBlockedCm : CONFIG.DEFAULT_FRONT_BLOCKED_CM,
        sideOpenCm: finite(opts.sideOpenCm) ? opts.sideOpenCm : CONFIG.DEFAULT_SIDE_OPEN_CM,
        sideWarnCm: CONFIG.SIDE_WARN_CM,

        // Sliding median buffers
        windowL: [],
        windowF: [],
        windowR: [],

        // Close-reading memory timestamps (ms)
        lastCloseL: 0,
        lastCloseF: 0,
        lastCloseR: 0,

        // Front 0-echo tracking
        frontZeroSince: null,
        lastNonZeroRawF: 0,
        lastPacketAt: 0,

        // Filtered output state
        cmL: null,
        cmF: null,
        cmR: null,
        rawL: 0,
        rawF: 0,
        rawR: 0,

        // Hysteresis states
        isFrontBlocked: false,
        isLeftOpen: false,
        isRightOpen: false,

        // Advice stability states
        currentAdvice: 'STRAIGHT',
        candidateAdvice: null,
        candidateCount: 0,
    };
}

// Reset sliding filter windows when N or mode changes
function resetFilterWindows(state) {
    state.windowL = [];
    state.windowF = [];
    state.windowR = [];
}

// Update configuration (from browser or server)
function updateSensorConfig(state, cfg = {}) {
    let changed = false;

    if (cfg.mode === 'raw' || cfg.mode === 'median') {
        if (state.mode !== cfg.mode) {
            state.mode = cfg.mode;
            changed = true;
        }
    }

    if (finite(cfg.n)) {
        const nextEff = normalizeN(cfg.n);
        if (state.effectiveN !== nextEff || state.n !== cfg.n) {
            state.n = cfg.n;
            state.effectiveN = nextEff;
            changed = true;
        }
    }

    if (typeof cfg.adviceEnabled === 'boolean') {
        state.adviceEnabled = cfg.adviceEnabled;
        if (!state.adviceEnabled) {
            state.currentAdvice = null;
            state.candidateAdvice = null;
            state.candidateCount = 0;
        }
    }

    if (finite(cfg.frontBlockedCm) && cfg.frontBlockedCm > 0) {
        state.frontBlockedCm = cfg.frontBlockedCm;
    }

    if (finite(cfg.sideOpenCm) && cfg.sideOpenCm > 0) {
        state.sideOpenCm = cfg.sideOpenCm;
    }

    if (changed) {
        resetFilterWindows(state);
    }
}

// Sliding median of an array where Infinity represents no-echo / far
function computeMedian(arr) {
    if (!arr || arr.length === 0) return null;
    const sorted = arr.slice().sort((a, b) => a - b);
    const mid = sorted[Math.floor(sorted.length / 2)];
    return mid === Infinity ? null : mid;
}

// Resolve reading with close-range memory rules
function resolveCloseReading(rawCm, lastCloseAt, now, cfg = CONFIG) {
    if (rawCm !== null) {
        if (rawCm < cfg.CLOSE_THRESHOLD_CM) {
            return { val: rawCm, isClose: true };
        }
        return { val: rawCm, isClose: false };
    }
    // No-echo (us === 0 or out of range)
    if (lastCloseAt > 0 && (now - lastCloseAt <= cfg.CLOSE_MEMORY_MS)) {
        // Within 500 ms of a close reading: treat as still close/blocked
        return { val: cfg.MIN_DIST_CM, isClose: false };
    }
    // True open space
    return { val: Infinity, isClose: false };
}

// Process an incoming parsed sensor packet
function processPacket(state, pkt, now = Date.now(), cfg = CONFIG) {
    state.lastPacketAt = now;
    state.rawL = pkt.leftUs;
    state.rawF = pkt.frontUs;
    state.rawR = pkt.rightUs;

    // Track front 0-echo duration
    if (pkt.frontUs === 0) {
        if (state.frontZeroSince === null) {
            state.frontZeroSince = now;
        }
    } else {
        state.frontZeroSince = null;
        state.lastNonZeroRawF = pkt.frontUs;
    }

    const rawLcm = usToCm(pkt.leftUs, cfg);
    const rawFcm = usToCm(pkt.frontUs, cfg);
    const rawRcm = usToCm(pkt.rightUs, cfg);

    // Apply close-range memory rules
    const resL = resolveCloseReading(rawLcm, state.lastCloseL, now, cfg);
    const resF = resolveCloseReading(rawFcm, state.lastCloseF, now, cfg);
    const resR = resolveCloseReading(rawRcm, state.lastCloseR, now, cfg);

    // Update close-reading memory timestamps only on actual readings
    if (rawLcm !== null) {
        state.lastCloseL = rawLcm < cfg.CLOSE_THRESHOLD_CM ? now : 0;
    }
    if (rawFcm !== null) {
        state.lastCloseF = rawFcm < cfg.CLOSE_THRESHOLD_CM ? now : 0;
    }
    if (rawRcm !== null) {
        state.lastCloseR = rawRcm < cfg.CLOSE_THRESHOLD_CM ? now : 0;
    }

    // Filter values based on mode
    if (state.mode === 'raw' || state.effectiveN <= 1) {
        state.cmL = resL.val === Infinity ? null : resL.val;
        state.cmF = resF.val === Infinity ? null : resF.val;
        state.cmR = resR.val === Infinity ? null : resR.val;
    } else {
        // Sliding window updated on EVERY packet
        state.windowL.push(resL.val);
        state.windowF.push(resF.val);
        state.windowR.push(resR.val);

        while (state.windowL.length > state.effectiveN) state.windowL.shift();
        while (state.windowF.length > state.effectiveN) state.windowF.shift();
        while (state.windowR.length > state.effectiveN) state.windowR.shift();

        state.cmL = computeMedian(state.windowL);
        state.cmF = computeMedian(state.windowF);
        state.cmR = computeMedian(state.windowR);
    }

    // ---- Hysteresis on Thresholds ----
    // 1. Front Blocked
    if (state.isFrontBlocked) {
        // To clear: must be >= FRONT_BLOCKED_CM + HYSTERESIS (or no-echo/null)
        if (state.cmF === null || state.cmF >= (state.frontBlockedCm + cfg.HYSTERESIS_CM)) {
            state.isFrontBlocked = false;
        }
    } else {
        // To become blocked: must be non-null and < FRONT_BLOCKED_CM
        if (state.cmF !== null && state.cmF < state.frontBlockedCm) {
            state.isFrontBlocked = true;
        }
    }

    // 2. Left Open
    if (state.isLeftOpen) {
        // To close: must be non-null and <= SIDE_OPEN_CM - HYSTERESIS
        if (state.cmL !== null && state.cmL <= (state.sideOpenCm - cfg.HYSTERESIS_CM)) {
            state.isLeftOpen = false;
        }
    } else {
        // To open: no echo (null) or > SIDE_OPEN_CM
        if (state.cmL === null || state.cmL > state.sideOpenCm) {
            state.isLeftOpen = true;
        }
    }

    // 3. Right Open
    if (state.isRightOpen) {
        // To close: must be non-null and <= SIDE_OPEN_CM - HYSTERESIS
        if (state.cmR !== null && state.cmR <= (state.sideOpenCm - cfg.HYSTERESIS_CM)) {
            state.isRightOpen = false;
        }
    } else {
        // To open: no echo (null) or > SIDE_OPEN_CM
        if (state.cmR === null || state.cmR > state.sideOpenCm) {
            state.isRightOpen = true;
        }
    }

    // Side warnings (< SIDE_WARN_CM)
    const sideWarnL = state.cmL !== null && state.cmL < state.sideWarnCm;
    const sideWarnR = state.cmR !== null && state.cmR < state.sideWarnCm;

    // ---- Advice Calculation ----
    let rawAdvice = null;
    if (state.adviceEnabled) {
        if (!state.isFrontBlocked) {
            rawAdvice = 'STRAIGHT';
        } else {
            if (state.isLeftOpen && !state.isRightOpen) {
                rawAdvice = 'TURN LEFT';
            } else if (!state.isLeftOpen && state.isRightOpen) {
                rawAdvice = 'TURN RIGHT';
            } else if (state.isLeftOpen && state.isRightOpen) {
                rawAdvice = 'LEFT or RIGHT';
            } else {
                rawAdvice = 'BLOCKED: REVERSE';
            }
        }

        // Becoming blocked must show immediately
        const wasBlockedAdvice = state.currentAdvice && state.currentAdvice !== 'STRAIGHT';
        const isNowBlockedAdvice = rawAdvice !== 'STRAIGHT';

        if (!state.currentAdvice) {
            state.currentAdvice = rawAdvice;
            state.candidateAdvice = null;
            state.candidateCount = 0;
        } else if (!wasBlockedAdvice && isNowBlockedAdvice) {
            state.currentAdvice = rawAdvice;
            state.candidateAdvice = null;
            state.candidateCount = 0;
        } else if (rawAdvice === state.currentAdvice) {
            state.candidateAdvice = null;
            state.candidateCount = 0;
        } else {
            // New advice must hold for 3 consecutive updates before replacing old one
            if (rawAdvice === state.candidateAdvice) {
                state.candidateCount++;
                if (state.candidateCount >= cfg.ADVICE_HOLD_COUNT) {
                    state.currentAdvice = rawAdvice;
                    state.candidateAdvice = null;
                    state.candidateCount = 0;
                }
            } else {
                state.candidateAdvice = rawAdvice;
                state.candidateCount = 1;
            }
        }
    } else {
        state.currentAdvice = null;
        state.candidateAdvice = null;
        state.candidateCount = 0;
    }

    const frontNoSignal = state.frontZeroSince !== null && (now - state.frontZeroSince >= cfg.FRONT_NO_SIGNAL_MS);

    return {
        mode: state.mode,
        n: state.effectiveN,
        inputN: state.n,
        cmL: state.cmL,
        cmF: state.cmF,
        cmR: state.cmR,
        rawL: state.rawL,
        rawF: state.rawF,
        rawR: state.rawR,
        frontBlocked: state.isFrontBlocked,
        leftOpen: state.isLeftOpen,
        rightOpen: state.isRightOpen,
        sideWarnL,
        sideWarnR,
        advice: state.currentAdvice,
        adviceChips: {
            leftOpen: state.isLeftOpen,
            rightOpen: state.isRightOpen,
        },
        frontNoSignal,
        lastNonZeroRawF: state.lastNonZeroRawF,
        ageMs: now - state.lastPacketAt
    };
}

module.exports = {
    CONFIG,
    usToCm,
    parseSensorPacket,
    normalizeN,
    createSensorState,
    updateSensorConfig,
    computeMedian,
    resetFilterWindows,
    processPacket,
};
