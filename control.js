'use strict';
// ============================================================================
//  control.js - ALL robot calculation lives here (pure functions, no I/O).
//
//  Pipeline:
//    1. global speed G (0..255)  (+ optional BOOST while the boost key is held)
//    2. straight-line trim (alpha): the FASTER side runs at G * alpha,
//       the slower side runs at G (maxed out)
//    3. arcade mixing:
//         Up/Down only            -> straight
//         Left/Right only         -> spin in place: 255 * spin strength (0..1) per side, NOT tied to global speed
//         Up/Down + Left/Right    -> arc: inner wheel = corrected * (1 - 2t)
//    4. dead-band, optional acceleration ramp
//    5. 6-byte UDP frame for the ESP32 (the ESP32 only applies it)
// ============================================================================

const CONFIG = {
    SPIN_BASE: 255,       // spin-in-place base speed, independent of the global speed (spin = SPIN_BASE * strength)
    SPIN_SCALE: 1.0,      // extra overall multiplier for spin (1.0 = leave as is)
    RAMP_PER_SEC: 3000,   // PWM units per second when "smooth" is on (3000 = 0 -> full in ~85 ms)
    MIN_PWM: 0,           // motor dead-band: any non-zero command is remapped to MIN_PWM..255 (0 = off)
    MAX_PWM: 255,         // hard cap on PWM (lower it, e.g. 191, to protect 9 V motors on a 12 V battery)
    BOOST_SPEED: 255      // speed used while the boost key is held (if higher than the chosen speed)
};

const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
const finite = (v) => typeof v === 'number' && Number.isFinite(v);
const sgn = (v) => (finite(v) ? (v > 0 ? 1 : v < 0 ? -1 : 0) : 0);
const num = (v, lo, hi, dflt) => (finite(v) ? clamp(v, lo, hi) : dflt);

// symmetric round (half away from zero) + clamp to the PWM cap; never returns -0
function fin(v, cfg = CONFIG) {
    const cap = cfg.MAX_PWM;
    v = clamp(v, -cap, cap);
    const m = Math.round(Math.abs(v));
    return m === 0 ? 0 : (v < 0 ? -m : m);
}

// Used when the browser goes quiet: stop and brake.
const NEUTRAL = Object.freeze({
    thr: 0, str: 0, speed: 0, turn: 0,
    boost: false, estop: false, smooth: false, brake: true
});
// spinLeft / spinRight: strength (0..1) of spin-in-place when the Left / Right key is pressed alone.
// spinInvert: flips the physical rotation direction of spin-in-place (Left key spins the other way).
const DEFAULT_CALIB = Object.freeze({
    alpha: 1, fasterSide: 'left',
    spinLeft: 1, spinRight: 1, spinInvert: false
});

// ---- Stage 1: speed actually requested (boost lifts it) -----------------------
function effectiveSpeed(input, cfg = CONFIG) {
    const s = clamp(finite(input.speed) ? input.speed : 0, 0, 255);
    return input.boost ? Math.max(s, cfg.BOOST_SPEED) : s;
}

// ---- Stage 2: corrected left/right speeds --------------------------------------
function globalSides(G, calib) {
    G = clamp(finite(G) ? G : 0, 0, 255);
    const alpha = clamp(finite(calib.alpha) ? calib.alpha : 1, 0, 1);
    return calib.fasterSide === 'right'
        ? { left: G, right: G * alpha }
        : { left: G * alpha, right: G };
}

// ---- Stage 3: arcade mixing -----------------------------------------------------
// input: { thr: -1|0|1, str: -1|0|1 (left|none|right), speed, turn, boost }
// returns [left, right] signed PWM, -255..255 (positive = forward)
function mix(input, calib, cfg = CONFIG) {
    const { left: gl, right: gr } = globalSides(effectiveSpeed(input, cfg), calib);
    const thr = input.thr, str = input.str;

    if (thr === 0 && str === 0) return [0, 0];

    if (thr === 0) {                                   // spin in place
        // The strength follows the KEY that is pressed (Left key -> spinLeft, Right key -> spinRight).
        const strength = clamp(finite(str > 0 ? calib.spinRight : calib.spinLeft)
                                   ? (str > 0 ? calib.spinRight : calib.spinLeft) : 1, 0, 1);
        // Spin ignores the global speed: base is always SPIN_BASE (255), then strength 0..1 is applied.
        // The alpha trim is still applied to that base so both sides spin evenly.
        const { left: sl, right: sr } = globalSides(cfg.SPIN_BASE, calib);
        const s = cfg.SPIN_SCALE * strength;
        // spinInvert flips only the rotation direction, not which slider is used.
        const clockwise = (str > 0) !== (calib.spinInvert === true);
        return clockwise ? [fin(sl * s, cfg), fin(-sr * s, cfg)]
                         : [fin(-sl * s, cfg), fin(sr * s, cfg)];
    }

    const k = 1 - 2 * clamp(finite(input.turn) ? input.turn : 0, 0, 1);
    let l = thr * gl;
    let r = thr * gr;
    if (str < 0) l *= k;                               // Left  -> left side is the inner wheel
    else if (str > 0) r *= k;                          // Right -> right side is the inner wheel
    return [fin(l, cfg), fin(r, cfg)];
}

// ---- Stage 4 ----------------------------------------------------------------------
function applyDeadband(v, cfg = CONFIG) {
    const minPwm = cfg.MIN_PWM;
    if (v === 0 || minPwm <= 0) return v;
    const mapped = Math.round(minPwm + (Math.abs(v) - 1) * (cfg.MAX_PWM - minPwm) / (cfg.MAX_PWM - 1));
    return fin(Math.sign(v) * mapped, cfg);
}

// Move cur toward target by at most `step` (float). Slowing down / stopping is instant;
// reversing passes through zero.
function ramp(cur, target, step) {
    if (target === 0) return 0;
    if (cur !== 0 && Math.sign(cur) !== Math.sign(target)) {
        return cur - Math.sign(cur) * Math.min(Math.abs(cur), step);
    }
    if (Math.abs(target) > Math.abs(cur)) {
        return Math.sign(target) * Math.min(Math.abs(target), Math.abs(cur) + step);
    }
    return target;
}

// ---- Stage 5: wire frame -----------------------------------------------------------
// [0xA5, dirL, pwmL, dirR, pwmR, xor(bytes 0..4)]
// dir: 0 = coast, 1 = forward, 2 = reverse, 3 = brake (both L298N inputs high)
const FRAME_HEADER = 0xA5;
const DIR_COAST = 0, DIR_FWD = 1, DIR_REV = 2, DIR_BRAKE = 3;

function sideBytes(v, brake) {
    if (v > 0) return [DIR_FWD, v];
    if (v < 0) return [DIR_REV, -v];
    return brake ? [DIR_BRAKE, 255] : [DIR_COAST, 0];
}

// `brake` only applies to a side whose value is 0.
function toFrame(left, right, brake = false) {
    const l = fin(left), r = fin(right);
    const [dl, pl] = sideBytes(l, brake);
    const [dr, pr] = sideBytes(r, brake);
    const b = [FRAME_HEADER, dl, pl, dr, pr];
    b.push(b[0] ^ b[1] ^ b[2] ^ b[3] ^ b[4]);
    return Buffer.from(b);
}

// For tests / debugging. Returns { l, r, brakeL, brakeR } or null if the frame is invalid.
function fromFrame(buf) {
    if (!buf || buf.length !== 6 || buf[0] !== FRAME_HEADER) return null;
    if ((buf[0] ^ buf[1] ^ buf[2] ^ buf[3] ^ buf[4]) !== buf[5]) return null;
    if (buf[1] > DIR_BRAKE || buf[3] > DIR_BRAKE) return null;
    const val = (d, p) => (d === DIR_FWD ? p : d === DIR_REV ? -p : 0);
    return {
        l: val(buf[1], buf[2]) || 0,
        r: val(buf[3], buf[4]) || 0,
        brakeL: buf[1] === DIR_BRAKE,
        brakeR: buf[3] === DIR_BRAKE
    };
}

// ---- Input validation (everything from the browser is untrusted) ---------------------
function sanitizeIntent(msg) {
    msg = msg || {};
    return {
        thr: sgn(msg.thr),
        str: sgn(msg.str),
        speed: Math.round(num(msg.speed, 0, 255, 0)),
        turn: num(msg.turn, 0, 1, 0),
        boost: msg.boost === true,
        estop: msg.estop === true,
        smooth: msg.smooth === true,
        brake: msg.brake === true
    };
}

function sanitizeCalib(msg, current) {
    msg = msg || {};
    const cur = current || DEFAULT_CALIB;
    return {
        alpha: Math.round(num(msg.alpha, 0, 1, cur.alpha) * 1000) / 1000,
        fasterSide: (msg.fasterSide === 'left' || msg.fasterSide === 'right') ? msg.fasterSide : cur.fasterSide,
        spinLeft:  Math.round(num(msg.spinLeft,  0, 1, finite(cur.spinLeft)  ? cur.spinLeft  : 1) * 1000) / 1000,
        spinRight: Math.round(num(msg.spinRight, 0, 1, finite(cur.spinRight) ? cur.spinRight : 1) * 1000) / 1000,
        spinInvert: typeof msg.spinInvert === 'boolean' ? msg.spinInvert : cur.spinInvert === true
    };
}

module.exports = {
    CONFIG, NEUTRAL, DEFAULT_CALIB,
    effectiveSpeed, globalSides, mix, applyDeadband, ramp,
    toFrame, fromFrame, sanitizeIntent, sanitizeCalib
};
