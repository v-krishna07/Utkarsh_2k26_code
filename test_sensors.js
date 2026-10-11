'use strict';
// ============================================================================
//  test_sensors.js - Comprehensive test suite for HC-SR04 parsing, filtering,
//  advice engine, focus isolation, integration, and soccer control regression.
// ============================================================================
const assert = require('assert');
const http = require('http');
const dgram = require('dgram');
const WebSocket = require('ws');
const sensors = require('./sensors');
const control = require('./control');
const server = require('./server');

let passedTests = 0;
let totalTests = 0;

function it(desc, fn) {
    totalTests++;
    try {
        fn();
        passedTests++;
        console.log(`  ✓ ${desc}`);
    } catch (err) {
        console.error(`  ✗ ${desc}`);
        console.error(err);
        process.exitCode = 1;
    }
}

async function itAsync(desc, fn) {
    totalTests++;
    try {
        await fn();
        passedTests++;
        console.log(`  ✓ ${desc}`);
    } catch (err) {
        console.error(`  ✗ ${desc}`);
        console.error(err);
        process.exitCode = 1;
    }
}

// Helper to construct a valid 9-byte sensor packet
function buildSensorBuffer(seq, leftUs, frontUs, rightUs, opts = {}) {
    const buf = Buffer.alloc(9);
    buf[0] = opts.header !== undefined ? opts.header : 0x5A;
    buf[1] = seq & 0xFF;
    buf[2] = leftUs & 0xFF;
    buf[3] = (leftUs >> 8) & 0xFF;
    buf[4] = frontUs & 0xFF;
    buf[5] = (frontUs >> 8) & 0xFF;
    buf[6] = rightUs & 0xFF;
    buf[7] = (rightUs >> 8) & 0xFF;

    if (opts.corruptXor) {
        buf[8] = 0x00;
    } else {
        let x = 0;
        for (let i = 0; i < 8; i++) x ^= buf[i];
        buf[8] = x;
    }
    return buf;
}

function cmToUs(cm) {
    return Math.round((cm * 2) / sensors.CONFIG.SPEED_OF_SOUND_CM_PER_US);
}

// ============================================================================
// TEST SUITE
// ============================================================================
async function runAllTests() {
    console.log('\n========================================');
    console.log(' RUNNING SENSOR & CONTROL TEST SUITE');
    console.log('========================================\n');

    // ------------------------------------------------------------------------
    // 1. Parsing
    // ------------------------------------------------------------------------
    console.log('1. PARSING TESTS');
    it('distinct L, F, R values land in the right boxes with little-endian uint16', () => {
        const leftUs = 1200;   // 0x04B0
        const frontUs = 2500;  // 0x09C4
        const rightUs = 4300;  // 0x10CC
        const buf = buildSensorBuffer(42, leftUs, frontUs, rightUs);

        const parsed = sensors.parseSensorPacket(buf);
        assert.ok(parsed, 'Packet should parse successfully');
        assert.strictEqual(parsed.seq, 42, 'Sequence number should match');
        assert.strictEqual(parsed.leftUs, leftUs, 'Left microseconds must match');
        assert.strictEqual(parsed.frontUs, frontUs, 'Front microseconds must match');
        assert.strictEqual(parsed.rightUs, rightUs, 'Right microseconds must match');
    });

    it('rejects packet with bad XOR checksum', () => {
        const buf = buildSensorBuffer(1, 1000, 2000, 3000, { corruptXor: true });
        const parsed = sensors.parseSensorPacket(buf);
        assert.strictEqual(parsed, null, 'Bad XOR checksum must return null');
    });

    it('rejects packet with incorrect header byte', () => {
        const buf = buildSensorBuffer(1, 1000, 2000, 3000, { header: 0xA5 });
        const parsed = sensors.parseSensorPacket(buf);
        assert.strictEqual(parsed, null, 'Wrong header must return null');
    });

    it('rejects packet with wrong length', () => {
        const shortBuf = Buffer.alloc(8);
        const longBuf = Buffer.alloc(10);
        assert.strictEqual(sensors.parseSensorPacket(shortBuf), null);
        assert.strictEqual(sensors.parseSensorPacket(longBuf), null);
        assert.strictEqual(sensors.parseSensorPacket(null), null);
    });

    // ------------------------------------------------------------------------
    // 2. Median & N Filtering
    // ------------------------------------------------------------------------
    console.log('\n2. MEDIAN & N FILTERING TESTS');
    it('normalizes N correctly: N=1, 5, 25; even N rounds up to next odd', () => {
        assert.strictEqual(sensors.normalizeN(1), 1);
        assert.strictEqual(sensors.normalizeN(5), 5);
        assert.strictEqual(sensors.normalizeN(25), 25);
        assert.strictEqual(sensors.normalizeN(2), 3, 'Even 2 should round up to 3');
        assert.strictEqual(sensors.normalizeN(4), 5, 'Even 4 should round up to 5');
        assert.strictEqual(sensors.normalizeN(6), 7, 'Even 6 should round up to 7');
        assert.strictEqual(sensors.normalizeN(24), 25, 'Even 24 should round up to 25');
        assert.strictEqual(sensors.normalizeN(0), 1, 'Below range clamps to 1');
        assert.strictEqual(sensors.normalizeN(30), 25, 'Above range clamps to 25');
    });

    it('single spike (0 or 200) among 5 values does not change the median', () => {
        // Compute median on array directly
        assert.strictEqual(sensors.computeMedian([15, 15, 15, 15, Infinity]), 15, '0-echo spike (Infinity) ignored');
        assert.strictEqual(sensors.computeMedian([15, 15, 15, 15, 80]), 15, 'High spike ignored');
        assert.strictEqual(sensors.computeMedian([15, 15, 15, 15, 2.5]), 15, 'Low spike ignored');

        // Test through state sliding window
        const state = sensors.createSensorState({ mode: 'median', n: 5 });
        let t = 1000;
        // 4 baseline packets of 15 cm
        for (let i = 0; i < 4; i++) {
            const pkt = { seq: i, leftUs: cmToUs(15), frontUs: cmToUs(15), rightUs: cmToUs(15) };
            sensors.processPacket(state, pkt, t += 30);
        }
        // 5th packet with spike (front sensor no echo 0 us -> Infinity)
        const spikePkt = { seq: 4, leftUs: cmToUs(15), frontUs: 0, rightUs: cmToUs(15) };
        const out = sensors.processPacket(state, spikePkt, t += 30);
        assert.strictEqual(Math.round(out.cmF), 15, 'Median front reading must remain 15 cm despite spike');
    });

    it('sliding window updates on EVERY packet without waiting for N new readings', () => {
        const state = sensors.createSensorState({ mode: 'median', n: 5 });
        let t = 1000;
        const out1 = sensors.processPacket(state, { seq: 1, leftUs: cmToUs(10), frontUs: cmToUs(20), rightUs: cmToUs(30) }, t);
        assert.strictEqual(Math.round(out1.cmF), 20, 'First packet produces output immediately');
        assert.strictEqual(state.windowF.length, 1, 'Window has 1 element');

        const out2 = sensors.processPacket(state, { seq: 2, leftUs: cmToUs(10), frontUs: cmToUs(22), rightUs: cmToUs(30) }, t + 30);
        assert.strictEqual(state.windowF.length, 2, 'Window updated on 2nd packet');
        assert.ok(out2.cmF !== null, 'Output exists on 2nd packet');
    });

    it('changing N or mode clears sliding filter windows', () => {
        const state = sensors.createSensorState({ mode: 'median', n: 5 });
        sensors.processPacket(state, { seq: 1, leftUs: cmToUs(15), frontUs: cmToUs(15), rightUs: cmToUs(15) }, 1000);
        assert.strictEqual(state.windowF.length, 1);

        sensors.updateSensorConfig(state, { n: 7 });
        assert.strictEqual(state.windowF.length, 0, 'Changing N should clear windows');
        assert.strictEqual(state.effectiveN, 7);

        sensors.processPacket(state, { seq: 2, leftUs: cmToUs(15), frontUs: cmToUs(15), rightUs: cmToUs(15) }, 1030);
        assert.strictEqual(state.windowF.length, 1);

        sensors.updateSensorConfig(state, { mode: 'raw' });
        assert.strictEqual(state.windowF.length, 0, 'Changing mode should clear windows');
    });

    it('N=1 behaves like raw mode', () => {
        const state = sensors.createSensorState({ mode: 'median', n: 1 });
        const out = sensors.processPacket(state, { seq: 1, leftUs: cmToUs(14), frontUs: cmToUs(18), rightUs: cmToUs(28) }, 1000);
        assert.strictEqual(Math.round(out.cmF), 18, 'N=1 behaves like raw');
    });

    // ------------------------------------------------------------------------
    // 3. Close-Range Rules & No-Echo Handling
    // ------------------------------------------------------------------------
    console.log('\n3. CLOSE-RANGE RULES & NO-ECHO TESTS');
    it('reading under 2.5 cm is clamped to 2.5 cm and counts as blocked', () => {
        const tinyEchoUs = 50; // 50 * 0.0343 / 2 = 0.857 cm
        const cm = sensors.usToCm(tinyEchoUs);
        assert.strictEqual(cm, 2.5, 'Reading under 2.5 cm must clamp to 2.5 cm');

        const state = sensors.createSensorState({ mode: 'raw' });
        const out = sensors.processPacket(state, { seq: 1, leftUs: cmToUs(30), frontUs: tinyEchoUs, rightUs: cmToUs(30) }, 1000);
        assert.strictEqual(out.cmF, 2.5, 'cmF clamped to 2.5');
        assert.strictEqual(out.frontBlocked, true, 'cmF 2.5 cm must be frontBlocked');
    });

    it('no-echo right after a close reading (< 20 cm) counts as still close for up to 500 ms', () => {
        const state = sensors.createSensorState({ mode: 'raw' });
        // t=1000: close reading of 10 cm
        sensors.processPacket(state, { seq: 1, leftUs: cmToUs(30), frontUs: cmToUs(10), rightUs: cmToUs(30) }, 1000);

        // t=1200 (200 ms later): no-echo (0 us)
        const out200 = sensors.processPacket(state, { seq: 2, leftUs: cmToUs(30), frontUs: 0, rightUs: cmToUs(30) }, 1200);
        assert.strictEqual(out200.cmF, 2.5, 'No-echo at 200 ms must clamp to 2.5 cm');
        assert.strictEqual(out200.frontBlocked, true, 'Still counted as blocked at 200 ms');

        // t=1450 (450 ms later): still within 500 ms
        const out450 = sensors.processPacket(state, { seq: 3, leftUs: cmToUs(30), frontUs: 0, rightUs: cmToUs(30) }, 1450);
        assert.strictEqual(out450.cmF, 2.5, 'No-echo at 450 ms must still clamp to 2.5 cm');

        // t=1550 (550 ms later): 500 ms close memory expired!
        const out550 = sensors.processPacket(state, { seq: 4, leftUs: cmToUs(30), frontUs: 0, rightUs: cmToUs(30) }, 1550);
        assert.strictEqual(out550.cmF, null, 'After 500 ms, no-echo becomes open space (null / >100)');
        assert.strictEqual(out550.frontBlocked, false, 'Front is no longer blocked');
    });

    it('only a no-echo after far readings means open space immediately', () => {
        const state = sensors.createSensorState({ mode: 'raw' });
        // t=2000: far reading of 40 cm (>= 20 cm)
        sensors.processPacket(state, { seq: 1, leftUs: cmToUs(30), frontUs: cmToUs(40), rightUs: cmToUs(30) }, 2000);

        // t=2050 (50 ms later): no echo
        const out = sensors.processPacket(state, { seq: 2, leftUs: cmToUs(30), frontUs: 0, rightUs: cmToUs(30) }, 2050);
        assert.strictEqual(out.cmF, null, 'No-echo after far reading immediately means open space');
        assert.strictEqual(out.frontBlocked, false);
    });

    it('front sensor: no signal wiring warning after 2 seconds of raw 0 us', () => {
        const state = sensors.createSensorState({ mode: 'raw' });
        let out = sensors.processPacket(state, { seq: 1, leftUs: cmToUs(30), frontUs: 0, rightUs: cmToUs(30) }, 1000);
        assert.strictEqual(out.frontNoSignal, false, 'Not 2 s yet');

        out = sensors.processPacket(state, { seq: 2, leftUs: cmToUs(30), frontUs: 0, rightUs: cmToUs(30) }, 2500);
        assert.strictEqual(out.frontNoSignal, false, '1.5 s elapsed: not 2 s yet');

        out = sensors.processPacket(state, { seq: 3, leftUs: cmToUs(30), frontUs: 0, rightUs: cmToUs(30) }, 3050);
        assert.strictEqual(out.frontNoSignal, true, '2.05 s of continuous 0 us must trigger wiring warning');
    });

    // ------------------------------------------------------------------------
    // 4. Advice Table, Hysteresis & 3-Update Hold
    // ------------------------------------------------------------------------
    console.log('\n4. ADVICE TABLE, HYSTERESIS & HOLD TESTS');
    it('produces STRAIGHT when front is clear', () => {
        const state = sensors.createSensorState({ mode: 'raw', adviceEnabled: true });
        const out = sensors.processPacket(state, {
            seq: 1, leftUs: cmToUs(30), frontUs: cmToUs(40), rightUs: cmToUs(30)
        }, 1000);
        assert.strictEqual(out.advice, 'STRAIGHT');
    });

    it('produces TURN LEFT when front is blocked and only left is open', () => {
        const state = sensors.createSensorState({ mode: 'raw', adviceEnabled: true });
        // F=10 (blocked < 12), L=30 (open > 25), R=15 (closed <= 25)
        const out = sensors.processPacket(state, {
            seq: 1, leftUs: cmToUs(30), frontUs: cmToUs(10), rightUs: cmToUs(15)
        }, 1000);
        assert.strictEqual(out.advice, 'TURN LEFT');
    });

    it('produces TURN RIGHT when front is blocked and only right is open', () => {
        const state = sensors.createSensorState({ mode: 'raw', adviceEnabled: true });
        // F=10 (blocked), L=15 (closed), R=30 (open)
        const out = sensors.processPacket(state, {
            seq: 1, leftUs: cmToUs(15), frontUs: cmToUs(10), rightUs: cmToUs(30)
        }, 1000);
        assert.strictEqual(out.advice, 'TURN RIGHT');
    });

    it('produces LEFT or RIGHT when front is blocked and both are open', () => {
        const state = sensors.createSensorState({ mode: 'raw', adviceEnabled: true });
        // F=10 (blocked), L=30 (open), R=30 (open)
        const out = sensors.processPacket(state, {
            seq: 1, leftUs: cmToUs(30), frontUs: cmToUs(10), rightUs: cmToUs(30)
        }, 1000);
        assert.strictEqual(out.advice, 'LEFT or RIGHT');
    });

    it('produces BLOCKED: REVERSE when front is blocked and neither side is open', () => {
        const state = sensors.createSensorState({ mode: 'raw', adviceEnabled: true });
        // F=10 (blocked), L=15 (closed), R=15 (closed)
        const out = sensors.processPacket(state, {
            seq: 1, leftUs: cmToUs(15), frontUs: cmToUs(10), rightUs: cmToUs(15)
        }, 1000);
        assert.strictEqual(out.advice, 'BLOCKED: REVERSE');
    });

    it('becoming blocked shows immediately on the very first update', () => {
        const state = sensors.createSensorState({ mode: 'raw', adviceEnabled: true });
        // Initially clear: STRAIGHT
        sensors.processPacket(state, { seq: 1, leftUs: cmToUs(30), frontUs: cmToUs(50), rightUs: cmToUs(15) }, 1000);
        assert.strictEqual(state.currentAdvice, 'STRAIGHT');

        // Front suddenly becomes blocked: should immediately show TURN LEFT without 3 updates
        const out = sensors.processPacket(state, { seq: 2, leftUs: cmToUs(30), frontUs: cmToUs(8), rightUs: cmToUs(15) }, 1030);
        assert.strictEqual(out.advice, 'TURN LEFT', 'Becoming blocked must transition immediately on first update');
    });

    it('requires 3 consecutive updates before replacing old advice when not becoming blocked', () => {
        const state = sensors.createSensorState({ mode: 'raw', adviceEnabled: true });
        // Establish BLOCKED: REVERSE
        sensors.processPacket(state, { seq: 1, leftUs: cmToUs(15), frontUs: cmToUs(8), rightUs: cmToUs(15) }, 1000);
        assert.strictEqual(state.currentAdvice, 'BLOCKED: REVERSE');

        // Now left side opens (candidate TURN LEFT)
        const pktTurnLeft = { seq: 2, leftUs: cmToUs(30), frontUs: cmToUs(8), rightUs: cmToUs(15) };

        // Update 1
        let out = sensors.processPacket(state, pktTurnLeft, 1030);
        assert.strictEqual(out.advice, 'BLOCKED: REVERSE', 'Update 1: holds old advice');

        // Update 2
        out = sensors.processPacket(state, pktTurnLeft, 1060);
        assert.strictEqual(out.advice, 'BLOCKED: REVERSE', 'Update 2: holds old advice');

        // Update 3: now it replaces old advice!
        out = sensors.processPacket(state, pktTurnLeft, 1090);
        assert.strictEqual(out.advice, 'TURN LEFT', 'Update 3: advice replaces old one');
    });

    it('3 cm hysteresis on frontBlocked and sideOpen thresholds', () => {
        const state = sensors.createSensorState({ mode: 'raw' });
        // frontBlockedCm = 12, hysteresis = 3 => unblocks at >= 15 cm
        // sideOpenCm = 25, hysteresis = 3 => closes at <= 22 cm

        // Start with front clear (16 cm) and left closed (20 cm)
        sensors.processPacket(state, { seq: 1, leftUs: cmToUs(20), frontUs: cmToUs(16), rightUs: cmToUs(20) }, 1000);
        assert.strictEqual(state.isFrontBlocked, false);
        assert.strictEqual(state.isLeftOpen, false);

        // Front drops below 12 cm -> blocked
        sensors.processPacket(state, { seq: 2, leftUs: cmToUs(20), frontUs: cmToUs(11), rightUs: cmToUs(20) }, 1030);
        assert.strictEqual(state.isFrontBlocked, true, '11 cm < 12 cm: front blocked');

        // Front rises to 13.5 cm (between 12 and 15): remains blocked due to hysteresis!
        sensors.processPacket(state, { seq: 3, leftUs: cmToUs(20), frontUs: cmToUs(13.5), rightUs: cmToUs(20) }, 1060);
        assert.strictEqual(state.isFrontBlocked, true, '13.5 cm < 15 cm: still blocked by hysteresis');

        // Front rises to 15.5 cm: clears
        sensors.processPacket(state, { seq: 4, leftUs: cmToUs(20), frontUs: cmToUs(15.5), rightUs: cmToUs(20) }, 1090);
        assert.strictEqual(state.isFrontBlocked, false, '15.5 cm >= 15 cm: front clears');

        // Left rises to 26 cm -> becomes open (> 25 cm)
        sensors.processPacket(state, { seq: 5, leftUs: cmToUs(26), frontUs: cmToUs(20), rightUs: cmToUs(20) }, 1120);
        assert.strictEqual(state.isLeftOpen, true, '26 cm > 25 cm: left opens');

        // Left drops to 23.5 cm (between 25 and 22): remains open due to hysteresis!
        sensors.processPacket(state, { seq: 6, leftUs: cmToUs(23.5), frontUs: cmToUs(20), rightUs: cmToUs(20) }, 1150);
        assert.strictEqual(state.isLeftOpen, true, '23.5 cm > 22 cm: still open by hysteresis');

        // Left drops to 21 cm: closes (<= 22 cm)
        sensors.processPacket(state, { seq: 7, leftUs: cmToUs(21), frontUs: cmToUs(20), rightUs: cmToUs(20) }, 1180);
        assert.strictEqual(state.isLeftOpen, false, '21 cm <= 22 cm: left closes');
    });

    it('with Advice switch off, no advice is sent or shown', () => {
        const state = sensors.createSensorState({ mode: 'raw', adviceEnabled: false });
        const out = sensors.processPacket(state, {
            seq: 1, leftUs: cmToUs(30), frontUs: cmToUs(50), rightUs: cmToUs(30)
        }, 1000);
        assert.strictEqual(out.advice, null, 'No advice when switch is off');

        // Turn advice switch off dynamically via updateSensorConfig
        state.adviceEnabled = true;
        sensors.processPacket(state, { seq: 2, leftUs: cmToUs(30), frontUs: cmToUs(50), rightUs: cmToUs(30) }, 1030);
        assert.strictEqual(state.currentAdvice, 'STRAIGHT');

        sensors.updateSensorConfig(state, { adviceEnabled: false });
        assert.strictEqual(state.currentAdvice, null, 'Clears immediately on config change');
    });

    // ------------------------------------------------------------------------
    // 5. Input Focus Isolation (Typing in an input does not drive)
    // ------------------------------------------------------------------------
    console.log('\n5. INPUT FOCUS ISOLATION TESTS');
    it('focus isolation logic: typing in input ignores drive keys and produces neutral intent', () => {
        // Model the browser's focus & keyboard isolation logic implemented in dashboard.html
        let isInputFocused = false;
        let keyHeld = {};
        let sentIntents = [];

        function releaseAll() {
            keyHeld = {};
            sentIntents.push({ thr: 0, str: 0, brake: true });
        }

        function handleFocusIn(tagName) {
            if (tagName === 'INPUT' || tagName === 'SELECT' || tagName === 'TEXTAREA') {
                isInputFocused = true;
                releaseAll();
            }
        }

        function handleFocusOut(tagName) {
            if (tagName === 'INPUT' || tagName === 'SELECT' || tagName === 'TEXTAREA') {
                isInputFocused = false;
            }
        }

        function handleKeyDown(code) {
            if (isInputFocused) {
                // Ignore driving keys
                return;
            }
            if (code === 'KeyW') keyHeld['up'] = true;
            if (code === 'KeyS') keyHeld['down'] = true;
            if (code === 'KeyA') keyHeld['left'] = true;
            if (code === 'KeyD') keyHeld['right'] = true;
            sentIntents.push({
                thr: (keyHeld['up'] ? 1 : 0) - (keyHeld['down'] ? 1 : 0),
                str: (keyHeld['right'] ? 1 : 0) - (keyHeld['left'] ? 1 : 0)
            });
        }

        // 1. Initial drive key works
        handleKeyDown('KeyW');
        assert.strictEqual(sentIntents[sentIntents.length - 1].thr, 1);

        // 2. User focuses input N or threshold
        handleFocusIn('INPUT');
        assert.strictEqual(isInputFocused, true);
        assert.strictEqual(sentIntents[sentIntents.length - 1].thr, 0, 'Neutral intent sent on focus');

        // 3. User types W, A, S, D in the input box
        sentIntents = [];
        handleKeyDown('KeyW');
        handleKeyDown('KeyS');
        handleKeyDown('KeyA');
        handleKeyDown('KeyD');
        assert.strictEqual(sentIntents.length, 0, 'NO driving intents generated while input is focused');

        // Verify that control engine given neutral intent produces zero motor output
        const engine = server.createControlEngine({ alpha: 1, fasterSide: 'left', spinLeft: 1, spinRight: 1, spinInvert: false });
        engine.setIntent({ thr: 0, str: 0, speed: 255, turn: 0, boost: false, estop: false, smooth: false, brake: true }, 1000);
        const stepResult = engine.step(1000);
        assert.strictEqual(stepResult.l, 0);
        assert.strictEqual(stepResult.r, 0);
        assert.strictEqual(stepResult.brake, true);

        // 4. Focus out restores driving
        handleFocusOut('INPUT');
        assert.strictEqual(isInputFocused, false);
        handleKeyDown('KeyW');
        assert.strictEqual(sentIntents.length, 1);
        assert.strictEqual(sentIntents[0].thr, 1);
    });

    // ------------------------------------------------------------------------
    // 6. Integration Test (Simulated packet -> server -> dashboard WebSocket)
    // ------------------------------------------------------------------------
    console.log('\n6. INTEGRATION TEST');
    await itAsync('simulated UDP packet goes through server and WebSocket broadcasts exact sensor data', async () => {
        // In server.js, UDP message event is hooked into sensors.processPacket(sensorState, ...)
        // Let's test the end-to-end data pipeline:
        const testState = sensors.createSensorState({ mode: 'raw', adviceEnabled: true });

        // Simulate ESP32 sending a packet: Left=20 cm, Front=10 cm, Right=35 cm
        const leftUs = cmToUs(20);
        const frontUs = cmToUs(10);
        const rightUs = cmToUs(35);
        const rawBuf = buildSensorBuffer(101, leftUs, frontUs, rightUs);

        // 1. Packet parsing
        const parsedPkt = sensors.parseSensorPacket(rawBuf);
        assert.ok(parsedPkt);
        assert.strictEqual(parsedPkt.seq, 101);

        // 2. Server packet processing
        const now = Date.now();
        const proc = sensors.processPacket(testState, parsedPkt, now);

        // 3. Server WebSocket payload generator
        assert.strictEqual(Math.round(proc.cmL), 20);
        assert.strictEqual(Math.round(proc.cmF), 10);
        assert.strictEqual(Math.round(proc.cmR), 35);
        assert.strictEqual(proc.frontBlocked, true);
        assert.strictEqual(proc.leftOpen, false);
        assert.strictEqual(proc.rightOpen, true);
        assert.strictEqual(proc.advice, 'TURN RIGHT');

        // 4. Verify dashboard display formatting (fmtVal function from dashboard.html)
        function fmtVal(cm, stale) {
            if (stale) return 'NO DATA';
            if (cm === null || cm === undefined) return '>100';
            return Math.round(cm) + ' cm';
        }

        const dispL = fmtVal(proc.cmL, false);
        const dispF = fmtVal(proc.cmF, false);
        const dispR = fmtVal(proc.cmR, false);

        assert.strictEqual(dispL, '20 cm');
        assert.strictEqual(dispF, '10 cm');
        assert.strictEqual(dispR, '35 cm');

        // Stale test (e.g. no packet for > 500 ms)
        assert.strictEqual(fmtVal(proc.cmF, true), 'NO DATA');
        // No echo test
        assert.strictEqual(fmtVal(null, false), '>100');
    });

    // ------------------------------------------------------------------------
    // 7. Regression Test: Existing Soccer Controls & Motor Frames Unchanged
    // ------------------------------------------------------------------------
    console.log('\n7. REGRESSION TESTS (SOCCER CONTROLS & MOTOR FRAMES)');
    it('toFrame and fromFrame encode and decode 6-byte motor frames with XOR checksum', () => {
        // Forward
        const fFwd = control.toFrame(180, 180, false);
        assert.strictEqual(fFwd.length, 6);
        assert.strictEqual(fFwd[0], 0xA5);
        const decFwd = control.fromFrame(fFwd);
        assert.strictEqual(decFwd.l, 180);
        assert.strictEqual(decFwd.r, 180);
        assert.strictEqual(decFwd.brakeL, false);

        // Reverse
        const fRev = control.toFrame(-120, -120, false);
        const decRev = control.fromFrame(fRev);
        assert.strictEqual(decRev.l, -120);
        assert.strictEqual(decRev.r, -120);

        // Brake
        const fBrake = control.toFrame(0, 0, true);
        const decBrake = control.fromFrame(fBrake);
        assert.strictEqual(decBrake.l, 0);
        assert.strictEqual(decBrake.r, 0);
        assert.strictEqual(decBrake.brakeL, true);
        assert.strictEqual(decBrake.brakeR, true);

        // Corrupted frame
        fFwd[5] = fFwd[5] ^ 0xFF;
        assert.strictEqual(control.fromFrame(fFwd), null, 'Corrupt XOR rejected');
    });

    it('control.mix correctly performs arcade mixing, alpha trim, and spin-in-place', () => {
        const calib = { alpha: 0.9, fasterSide: 'left', spinLeft: 1, spinRight: 0.8, spinInvert: false };

        // 1. Straight drive: faster left side trimmed by alpha (180 * 0.9 = 162), right maxed (180)
        const straight = control.mix({ thr: 1, str: 0, speed: 180, turn: 0, boost: false }, calib);
        assert.strictEqual(straight[0], 162);
        assert.strictEqual(straight[1], 180);

        // 2. Spin in place (Left): thr=0, str=-1
        // Independent of global speed, base=255. Counter-clockwise: left negative, right positive
        const spinL = control.mix({ thr: 0, str: -1, speed: 100, turn: 0, boost: false }, calib);
        assert.strictEqual(spinL[0], -Math.round(255 * 0.9));
        assert.strictEqual(spinL[1], 255);

        // 3. Spin in place (Right): thr=0, str=1, uses spinRight=0.8
        const spinR = control.mix({ thr: 0, str: 1, speed: 100, turn: 0, boost: false }, calib);
        assert.strictEqual(spinR[0], Math.round(255 * 0.9 * 0.8));
        assert.strictEqual(spinR[1], -Math.round(255 * 0.8));

        // 4. Boost lifts speed to 255
        const boosted = control.mix({ thr: 1, str: 0, speed: 100, turn: 0, boost: true }, { ...calib, alpha: 1 });
        assert.strictEqual(boosted[0], 255);
        assert.strictEqual(boosted[1], 255);
    });

    it('control.ramp smooths acceleration and stops instantly', () => {
        const step = 50;
        let cur = 0;
        cur = control.ramp(cur, 200, step);
        assert.strictEqual(cur, 50);
        cur = control.ramp(cur, 200, step);
        assert.strictEqual(cur, 100);

        // Instant stop
        cur = control.ramp(cur, 0, step);
        assert.strictEqual(cur, 0, 'Stopping is instant');
    });

    it('createControlEngine handles neutral timeouts, deadband, and estop failsafe', () => {
        const calib = { alpha: 1, fasterSide: 'left', spinLeft: 1, spinRight: 1, spinInvert: false };
        const engine = server.createControlEngine(calib, { intentTimeoutMs: 100 });

        let t = 1000;
        engine.setIntent({ thr: 1, str: 0, speed: 200, turn: 0, boost: false, estop: false, smooth: false, brake: false }, t);
        let out = engine.step(t);
        assert.strictEqual(out.l, 200);
        assert.strictEqual(out.r, 200);
        assert.strictEqual(out.brake, false);

        // Timeout after 150 ms (> 100 ms intent timeout) -> failsafe brakes
        out = engine.step(t + 150);
        assert.strictEqual(out.l, 0);
        assert.strictEqual(out.r, 0);
        assert.strictEqual(out.brake, true, 'Intent timeout triggers brake');

        // E-stop engages brake immediately
        engine.setIntent({ thr: 1, str: 0, speed: 200, turn: 0, boost: false, estop: true, smooth: false, brake: false }, t + 200);
        out = engine.step(t + 200);
        assert.strictEqual(out.l, 0);
        assert.strictEqual(out.r, 0);
        assert.strictEqual(out.brake, true);
    });

    // ------------------------------------------------------------------------
    // SUMMARY
    // ------------------------------------------------------------------------
    console.log('\n========================================');
    console.log(` RESULTS: ${passedTests}/${totalTests} tests passed (${Math.round((passedTests / totalTests) * 100)}%)`);
    console.log('========================================\n');

    if (passedTests !== totalTests) {
        process.exit(1);
    } else {
        process.exit(0);
    }
}

if (require.main === module) {
    runAllTests().catch((err) => {
        console.error('Test suite error:', err);
        process.exit(1);
    });
}

module.exports = { runAllTests };
