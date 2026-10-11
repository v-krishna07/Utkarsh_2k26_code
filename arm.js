'use strict';
// ============================================================================
//  arm.js - 2-servo robotic arm controller (pure functions & arm engine).
//
//  Platform: ESP32 only (Core 3.x, LEDC 50 Hz, 14-bit).
//  All calculations run on the laptop; the ESP32 performs no math.
//
//  Servos:
//    - LIFT servo: moves DOWN and UP (GPIO 13). Keys: I = up, K = down.
//    - GRIP servo: rotates to grip   (GPIO 32). Keys: J = rotate one way, L = other way.
//
//  Arm frame format (6 bytes, UDP, port 4210):
//      [0] 0xA6            header
//      [1] lift_duty_lo    low byte
//      [2] lift_duty_hi    high byte
//      [3] grip_duty_lo    low byte
//      [4] grip_duty_hi    high byte
//      [5] xor             XOR of bytes 0..4
//
//  Math:
//      pulse_us = pulseMin + (angle / 180) * (pulseMax - pulseMin)
//      duty     = Math.round(pulse_us * 16384 / 20000)   (50 Hz, 14-bit LEDC)
// ============================================================================

const ARM_FRAME_HEADER = 0xA6;
const ARM_FRAME_LEN    = 6;

// Duty bounds for ESP32 validation (~427 µs to ~2685 µs at 50 Hz, 14-bit)
const ARM_DUTY_MIN = 350;
const ARM_DUTY_MAX = 2200;

const DEFAULT_CONFIG = Object.freeze({
    lift: Object.freeze({
        speed: 60,       // deg/s (10..180)
        min: 50,
        max: 150,
        home: 100,
        invert: false,
        pulseMin: 544,
        pulseMax: 2400,
    }),
    grip: Object.freeze({
        speed: 25,       // deg/s (5..120) - moves slower than lift
        min: 0,
        max: 180,
        home: 90,
        invert: false,
        pulseMin: 544,
        pulseMax: 2400,
    }),
    speedLift: 60,
    speedGrip: 25,
});

const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
const finite = (v) => typeof v === 'number' && Number.isFinite(v);

// Convert angle in degrees [0..180] to pulse width in microseconds
function angleToPulse(angle, jointCfg) {
    const a = clamp(finite(angle) ? angle : 0, 0, 180);
    const pMin = jointCfg && finite(jointCfg.pulseMin) ? jointCfg.pulseMin : 544;
    const pMax = jointCfg && finite(jointCfg.pulseMax) ? jointCfg.pulseMax : 2400;
    return pMin + (a / 180) * (pMax - pMin);
}

// Convert pulse width in microseconds to 14-bit LEDC duty count at 50 Hz (20,000 µs period)
function pulseToDuty(pulseUs) {
    if (!finite(pulseUs)) pulseUs = 1500;
    const duty = Math.round((pulseUs * 16384) / 20000);
    return clamp(duty, ARM_DUTY_MIN, ARM_DUTY_MAX);
}

// Full conversion from joint angle to duty
function angleToDuty(angle, jointCfg) {
    const pulse = angleToPulse(angle, jointCfg);
    return pulseToDuty(pulse);
}

// Encode 6-byte arm frame: [0xA6, lift_duty_lo, lift_duty_hi, grip_duty_lo, grip_duty_hi, xor]
function toArmFrame(liftDuty, gripDuty) {
    const lDuty = clamp(Math.round(finite(liftDuty) ? liftDuty : 1290), ARM_DUTY_MIN, ARM_DUTY_MAX);
    const gDuty = clamp(Math.round(finite(gripDuty) ? gripDuty : 1206), ARM_DUTY_MIN, ARM_DUTY_MAX);

    const buf = Buffer.alloc(ARM_FRAME_LEN);
    buf[0] = ARM_FRAME_HEADER;
    buf[1] = lDuty & 0xFF;
    buf[2] = (lDuty >> 8) & 0xFF;
    buf[3] = gDuty & 0xFF;
    buf[4] = (gDuty >> 8) & 0xFF;
    buf[5] = buf[0] ^ buf[1] ^ buf[2] ^ buf[3] ^ buf[4];
    return buf;
}

// Decode and validate 6-byte arm frame (returns { liftDuty, gripDuty } or null)
function fromArmFrame(buf) {
    if (!buf || buf.length !== ARM_FRAME_LEN || buf[0] !== ARM_FRAME_HEADER) return null;
    const xor = buf[0] ^ buf[1] ^ buf[2] ^ buf[3] ^ buf[4];
    if (xor !== buf[5]) return null;

    const liftDuty = buf[1] | (buf[2] << 8);
    const gripDuty = buf[3] | (buf[4] << 8);

    if (liftDuty < ARM_DUTY_MIN || liftDuty > ARM_DUTY_MAX) return null;
    if (gripDuty < ARM_DUTY_MIN || gripDuty > ARM_DUTY_MAX) return null;

    return { liftDuty, gripDuty };
}

// Sanitize joint configuration
function sanitizeJointConfig(userJoint, defJoint, defaultSpeed, minSpeed, maxSpeed) {
    const u = userJoint || {};
    const d = defJoint || {};

    let speed = finite(u.speed) ? clamp(Math.round(u.speed), minSpeed, maxSpeed) : (d.speed || defaultSpeed);

    let min = finite(u.min) ? clamp(Math.round(u.min), 0, 180) : d.min;
    let max = finite(u.max) ? clamp(Math.round(u.max), 0, 180) : d.max;
    if (min > max) {
        const tmp = min; min = max; max = tmp;
    }

    const home = finite(u.home) ? clamp(Math.round(u.home), min, max) : clamp(d.home, min, max);
    const invert = typeof u.invert === 'boolean' ? u.invert : !!d.invert;

    let pulseMin = finite(u.pulseMin) ? clamp(Math.round(u.pulseMin), 400, 2600) : (d.pulseMin || 544);
    let pulseMax = finite(u.pulseMax) ? clamp(Math.round(u.pulseMax), 400, 2600) : (d.pulseMax || 2400);
    if (pulseMin >= pulseMax) {
        pulseMin = 544;
        pulseMax = 2400;
    }

    return { speed, min, max, home, invert, pulseMin, pulseMax };
}

// Sanitize entire arm configuration
function sanitizeArmConfig(userCfg, fallbackCfg = DEFAULT_CONFIG) {
    const u = userCfg || {};
    const f = fallbackCfg || DEFAULT_CONFIG;

    // Check if speeds were provided at top level or inside joint objects
    let liftUser = { ...(u.lift || {}) };
    if (finite(u.speedLift)) liftUser.speed = u.speedLift;

    let gripUser = { ...(u.grip || {}) };
    if (finite(u.speedGrip)) gripUser.speed = u.speedGrip;

    const lift = sanitizeJointConfig(liftUser, f.lift, 60, 10, 180);
    const grip = sanitizeJointConfig(gripUser, f.grip, 25, 5, 120);

    return {
        lift,
        grip,
        speedLift: lift.speed,
        speedGrip: grip.speed,
    };
}

// Sanitize arm intent from browser (handles raw keys and direct directions, opposites cancellation)
function sanitizeArmIntent(msg) {
    const m = msg || {};
    const sgn = (v) => (v > 0 ? 1 : v < 0 ? -1 : 0);

    let liftDir = 0;
    let gripDir = 0;

    // Check explicit keys if provided
    const keyI = m.i === true;
    const keyK = m.k === true;
    const keyJ = m.j === true;
    const keyL = m.l === true;

    if (keyI && keyK) {
        liftDir = 0; // Opposite keys cancel
    } else if (keyI) {
        liftDir = 1;
    } else if (keyK) {
        liftDir = -1;
    } else if (finite(m.liftDir)) {
        liftDir = sgn(m.liftDir);
    }

    if (keyJ && keyL) {
        gripDir = 0; // Opposite keys cancel
    } else if (keyJ) {
        gripDir = -1;
    } else if (keyL) {
        gripDir = 1;
    } else if (finite(m.gripDir)) {
        gripDir = sgn(m.gripDir);
    }

    return {
        liftDir,
        gripDir,
        home: m.home === true,
        estop: m.estop === true,
    };
}

// Arm Engine (stateful step runner, 50 Hz simulation)
function createArmEngine(initialConfig = DEFAULT_CONFIG, options = {}) {
    let cfg = sanitizeArmConfig(initialConfig);
    const timeoutMs = options.intentTimeoutMs || 300;

    let liftAngle = cfg.lift.home;
    let gripAngle = cfg.grip.home;

    let lastStepAt = null;
    let intent = null;
    let intentAt = 0;

    let prevLiftDuty = -1;
    let prevGripDuty = -1;

    function reset() {
        intent = null;
        intentAt = 0;
        lastStepAt = null;
        liftAngle = cfg.lift.home;
        gripAngle = cfg.grip.home;
        prevLiftDuty = -1;
        prevGripDuty = -1;
    }

    function updateConfig(newCfg) {
        cfg = sanitizeArmConfig(newCfg, cfg);
        liftAngle = clamp(liftAngle, cfg.lift.min, cfg.lift.max);
        gripAngle = clamp(gripAngle, cfg.grip.min, cfg.grip.max);
    }

    function setIntent(newIntent, now = Date.now()) {
        intent = sanitizeArmIntent(newIntent);
        intentAt = now;
    }

    function step(now = Date.now()) {
        const dt = lastStepAt !== null ? Math.min(0.1, Math.max(0, (now - lastStepAt) / 1000)) : 0;
        lastStepAt = now;

        const isFresh = intent !== null && (now - intentAt) <= timeoutMs;
        const currentIntent = isFresh ? intent : { liftDir: 0, gripDir: 0, home: false, estop: false };

        let moving = false;

        if (currentIntent.estop) {
            // E-stop active: block all arm motion
            moving = false;
        } else if (currentIntent.home) {
            // Stow / HOME requested: return both to home poses
            if (liftAngle !== cfg.lift.home || gripAngle !== cfg.grip.home) {
                liftAngle = cfg.lift.home;
                gripAngle = cfg.grip.home;
                moving = true;
            }
        } else {
            // Lift joint motion
            const rawLiftDir = currentIntent.liftDir;
            if (rawLiftDir !== 0) {
                const effLiftDir = cfg.lift.invert ? -rawLiftDir : rawLiftDir;
                const nextLift = clamp(liftAngle + effLiftDir * cfg.lift.speed * dt, cfg.lift.min, cfg.lift.max);
                if (nextLift !== liftAngle) moving = true;
                liftAngle = nextLift;
            }

            // Grip joint motion (independent speed, moves slower)
            const rawGripDir = currentIntent.gripDir;
            if (rawGripDir !== 0) {
                const effGripDir = cfg.grip.invert ? -rawGripDir : rawGripDir;
                const nextGrip = clamp(gripAngle + effGripDir * cfg.grip.speed * dt, cfg.grip.min, cfg.grip.max);
                if (nextGrip !== gripAngle) moving = true;
                gripAngle = nextGrip;
            }
        }

        const liftDuty = angleToDuty(liftAngle, cfg.lift);
        const gripDuty = angleToDuty(gripAngle, cfg.grip);

        const dutyChanged = (liftDuty !== prevLiftDuty) || (gripDuty !== prevGripDuty);
        prevLiftDuty = liftDuty;
        prevGripDuty = gripDuty;

        return {
            liftAngle,
            gripAngle,
            liftDuty,
            gripDuty,
            moving,
            dutyChanged,
        };
    }

    return {
        reset,
        updateConfig,
        getConfig: () => ({ ...cfg }),
        setIntent,
        step,
        getLiftAngle: () => liftAngle,
        getGripAngle: () => gripAngle,
    };
}

module.exports = {
    ARM_FRAME_HEADER,
    ARM_FRAME_LEN,
    ARM_DUTY_MIN,
    ARM_DUTY_MAX,
    DEFAULT_CONFIG,
    angleToPulse,
    pulseToDuty,
    angleToDuty,
    toArmFrame,
    fromArmFrame,
    sanitizeArmConfig,
    sanitizeArmIntent,
    createArmEngine,
};
