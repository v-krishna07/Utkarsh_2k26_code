'use strict';
// ============================================================================
//  Soccer Bot & Maze Driver Controller Server:
//  Browser (intent/config, WebSocket) -> Node.js (ALL math) -> ESP32 (UDP)
//
//  Usage:   node server.js                  (ESP32 at 192.168.4.1, its own hotspot)
//           node server.js 192.168.4.1      (override ESP32 IP)
//           PORT=9000 node server.js        (override dashboard port)
//  Then open http://localhost:8080 in your browser.
// ============================================================================
const http = require('http');
const fs = require('fs');
const net = require('net');
const path = require('path');
const dgram = require('dgram');
const WebSocket = require('ws');
const control = require('./control');
const sensors = require('./sensors');

// ---------------------------- CONFIG ----------------------------------------
const ESP32_IP   = process.argv[2] || process.env.ESP32_IP || '192.168.4.1';
const ESP32_PORT = Number(process.env.ESP32_PORT) || 4210;
const PORT       = Number(process.env.PORT) || 8080;
const HOST       = process.env.HOST || '127.0.0.1';   // loopback only

const TICK_MS           = 20;    // heartbeat / ramp period (50 Hz)
const INTENT_TIMEOUT_MS = 300;   // no fresh intent from browser -> brake
const TELEMETRY_MS      = 50;    // dashboard motor telemetry refresh (20 Hz)
const SENSOR_TELEM_MS   = 40;    // sensor telemetry refresh (25 Hz)
const MAX_SEND_QUEUE    = 4;     // drop frame instead of queueing more than this many
const CALIB_FILE = process.env.CALIB_FILE || path.join(__dirname, 'calibration.json');

if (net.isIP(ESP32_IP) === 0) {
    console.error(`"${ESP32_IP}" is not a valid IP address.`);
    process.exit(1);
}

// ---------------------------- CALIBRATION (persisted) -----------------------
function loadCalib() {
    try {
        const j = JSON.parse(fs.readFileSync(CALIB_FILE, 'utf8'));
        return control.sanitizeCalib(j, control.DEFAULT_CALIB);
    } catch (_) {
        return { ...control.DEFAULT_CALIB };
    }
}
let saveTimer = null;
function saveCalib() {
    clearTimeout(saveTimer);
    saveTimer = setTimeout(() => {
        fs.writeFile(CALIB_FILE, JSON.stringify(calib, null, 2), (err) => {
            if (err) console.error('Could not save calibration:', err.message);
        });
    }, 300);
}
let calib = loadCalib();

// ---------------------------- SENSOR STATE ----------------------------------
const sensorState = sensors.createSensorState();

// ---------------------------- UDP -------------------------------------------
const udp = dgram.createSocket('udp4');
udp.on('error', (err) => console.error('UDP socket error:', err.message));

// Listen for sensor packets from the ESP32 (9-byte, header 0x5A)
udp.on('message', (msg) => {
    if (msg.length !== 9 || msg[0] !== 0x5A) return;
    const pkt = sensors.parseSensorPacket(msg);
    if (!pkt) return;
    sensors.processPacket(sensorState, pkt, Date.now());
});

udp.bind(0);

let lastUdpErrLog = 0;
let framesSent = 0, framesDropped = 0;

function udpSend(frame) {
    udp.send(frame, ESP32_PORT, ESP32_IP, (err) => {
        if (err && Date.now() - lastUdpErrLog > 2000) {
            lastUdpErrLog = Date.now();
            console.error('UDP send error:', err.message, '(is this laptop connected to the robot Wi-Fi?)');
        }
    });
}

function sendFrame(l, r, brake) {
    let queued = 0;
    try { queued = udp.getSendQueueSize(); } catch (_) {}
    if (queued > MAX_SEND_QUEUE) { framesDropped++; return; }
    framesSent++;
    udpSend(control.toFrame(l, r, brake));
}

function sendStopBurst() {
    const f = control.toFrame(0, 0, true);
    udpSend(f);
    setTimeout(() => udpSend(f), 20);
    setTimeout(() => udpSend(f), 40);
}

// ---------------------------- CONTROL ENGINE --------------------------------
// Testable without sockets: handles ramp, deadband, mixing, estop failsafe.
function createControlEngine(calibRef, options = {}) {
    let intent = null;
    let intentAt = 0;
    let curL = 0, curR = 0;
    let estopPrev = false;
    let waitNeutral = false;
    let lastStepAt = Date.now();
    const timeoutMs = options.intentTimeoutMs || INTENT_TIMEOUT_MS;

    function reset() {
        intent = null;
        curL = 0; curR = 0;
        estopPrev = false;
        waitNeutral = false;
    }

    function setIntent(newIntent, now) {
        intent = newIntent;
        intentAt = now;
    }

    function step(now, onFrame) {
        const dt = Math.min(0.1, Math.max(0, (now - lastStepAt) / 1000));
        lastStepAt = now;

        const fresh = intent !== null && (now - intentAt) <= timeoutMs;
        const i = fresh ? intent : control.NEUTRAL;

        if (estopPrev && !i.estop) waitNeutral = true;
        estopPrev = i.estop;
        if (waitNeutral && i.thr === 0 && i.str === 0) waitNeutral = false;

        let target = [0, 0];
        let brake = true;
        if (!i.estop && !waitNeutral) {
            target = control.mix(i, calibRef).map((v) => control.applyDeadband(v));
            brake = i.brake && target[0] === 0 && target[1] === 0;
        }

        if (i.estop || !i.smooth) {
            curL = target[0];
            curR = target[1];
        } else {
            const stepSize = control.CONFIG.RAMP_PER_SEC * dt;
            curL = control.ramp(curL, target[0], stepSize);
            curR = control.ramp(curR, target[1], stepSize);
        }

        const l = Math.round(curL), r = Math.round(curR);
        if (onFrame) onFrame(l, r, brake);
        return { l, r, brake, intent: i };
    }

    return {
        reset,
        setIntent,
        step,
        getCurL: () => curL,
        getCurR: () => curR,
        getIntent: () => intent,
    };
}

const engine = createControlEngine(calib);

// ---------------------------- HTTP (dashboard) ------------------------------
const LOOPBACK = HOST === '127.0.0.1' || HOST === 'localhost' || HOST === '::1';

function hostOk(req) {
    if (!LOOPBACK) return true;
    return /^(localhost|127\.0\.0\.1|\[::1\])(:\d+)?$/i.test(String(req.headers.host || ''));
}

function originOk(req) {
    const origin = req.headers.origin;
    if (!origin) return true;
    try {
        return new URL(origin).host === req.headers.host;
    } catch (_) {
        return false;
    }
}

const server = http.createServer((req, res) => {
    if (!hostOk(req)) {
        res.writeHead(403, { 'Content-Type': 'text/plain' });
        res.end('Forbidden');
        return;
    }
    const reqPath = String(req.url || '/').split('?')[0].split('#')[0];
    if (reqPath === '/' || reqPath === '/index.html' || reqPath === '/dashboard.html') {
        fs.readFile(path.join(__dirname, 'dashboard.html'), (err, data) => {
            if (err) {
                res.writeHead(404, { 'Content-Type': 'text/plain' });
                res.end('dashboard.html not found next to server.js');
                return;
            }
            res.writeHead(200, {
                'Content-Type': 'text/html; charset=utf-8',
                'Cache-Control': 'no-store'
            });
            res.end(data);
        });
    } else if (reqPath === '/favicon.ico') {
        res.writeHead(204);
        res.end();
    } else {
        res.writeHead(404, { 'Content-Type': 'text/plain' });
        res.end('Not found');
    }
});

server.on('error', (err) => {
    if (err.code === 'EADDRINUSE') {
        console.error(`Port ${PORT} is already in use. Close the other server or run with PORT=9000.`);
    } else {
        console.error('HTTP server error:', err.message);
    }
    process.exit(1);
});

// ---------------------------- WebSocket -------------------------------------
const wss = new WebSocket.Server({
    noServer: true,
    maxPayload: 2048,
    verifyClient: (info) => hostOk(info.req) && originOk(info.req)
});

server.on('upgrade', (req, socket, head) => {
    let pathname = '/';
    try {
        pathname = new URL(req.url, `http://${req.headers.host || 'localhost'}`).pathname;
    } catch (_) {
        pathname = String(req.url || '/').split('?')[0];
    }

    if (pathname === '/' || pathname === '/ws') {
        wss.handleUpgrade(req, socket, head, (ws) => {
            wss.emit('connection', ws, req);
        });
    } else {
        socket.destroy();
    }
});

let controller = null;
let lastTelemetryAt = 0;
let lastSensorTelemAt = 0;

function send(ws, obj) {
    if (ws && ws.readyState === WebSocket.OPEN && ws.bufferedAmount < 4096) {
        ws.send(JSON.stringify(obj));
    }
}

const configMsg = () => ({
    type: 'config',
    alpha: calib.alpha, fasterSide: calib.fasterSide,
    spinLeft: calib.spinLeft, spinRight: calib.spinRight, spinInvert: calib.spinInvert
});

const calibText = () =>
    `faster side = ${calib.fasterSide}, alpha = ${calib.alpha}, ` +
    `spin L = ${calib.spinLeft}, spin R = ${calib.spinRight}, spin reversed = ${calib.spinInvert}`;

function makeSensorPayload(now) {
    const ageMs = sensorState.lastPacketAt > 0 ? (now - sensorState.lastPacketAt) : 999999;
    const frontNoSignal = sensorState.frontZeroSince !== null && (now - sensorState.frontZeroSince >= sensors.CONFIG.FRONT_NO_SIGNAL_MS);

    return {
        type: 'sensors',
        mode: sensorState.mode,
        n: sensorState.effectiveN,
        inputN: sensorState.n,
        cmL: sensorState.cmL,
        cmF: sensorState.cmF,
        cmR: sensorState.cmR,
        rawL: sensorState.rawL,
        rawF: sensorState.rawF,
        rawR: sensorState.rawR,
        frontBlocked: sensorState.isFrontBlocked,
        leftOpen: sensorState.isLeftOpen,
        rightOpen: sensorState.isRightOpen,
        sideWarnL: sensorState.cmL !== null && sensorState.cmL < sensorState.sideWarnCm,
        sideWarnR: sensorState.cmR !== null && sensorState.cmR < sensorState.sideWarnCm,
        advice: sensorState.currentAdvice,
        adviceChips: {
            leftOpen: sensorState.isLeftOpen,
            rightOpen: sensorState.isRightOpen
        },
        frontNoSignal,
        lastNonZeroRawF: sensorState.lastNonZeroRawF,
        ageMs
    };
}

wss.on('connection', (ws) => {
    if (controller) {
        ws.close(4001, 'Another dashboard is already open');
        return;
    }
    controller = ws;
    engine.reset();
    console.log('>>> Dashboard connected.');
    send(ws, configMsg());
    send(ws, makeSensorPayload(Date.now()));

    ws.on('message', (raw) => {
        let msg;
        try { msg = JSON.parse(raw.toString()); } catch (_) { return; }
        if (!msg || typeof msg !== 'object') return;

        if (msg.type === 'intent') {
            const next = control.sanitizeIntent(msg);
            engine.setIntent(next, Date.now());
            engine.step(Date.now(), sendFrame);
        } else if (msg.type === 'calib') {
            calib = control.sanitizeCalib(msg, calib);
            saveCalib();
            send(ws, configMsg());
            engine.step(Date.now(), sendFrame);
            console.log('Calibration: ' + calibText());
        } else if (msg.type === 'sensorConfig') {
            sensors.updateSensorConfig(sensorState, msg);
            send(ws, makeSensorPayload(Date.now()));
        }
    });

    ws.on('close', () => {
        if (controller !== ws) return;
        controller = null;
        engine.reset();
        console.log('>>> Dashboard disconnected - sending STOP.');
        sendStopBurst();
    });

    ws.on('error', (err) => console.error('WebSocket error:', err.message));
});

// Periodic control, motor telemetry (20 Hz), and sensor telemetry (25 Hz)
setInterval(() => {
    const now = Date.now();

    // Run motor control step
    const out = engine.step(now, sendFrame);

    // Motor telemetry to dashboard (20 Hz)
    if (controller && now - lastTelemetryAt >= TELEMETRY_MS) {
        lastTelemetryAt = now;
        const i = engine.getIntent() || control.NEUTRAL;
        const g = control.globalSides(control.effectiveSpeed(i), calib);
        send(controller, {
            type: 'telemetry',
            l: out.l, r: out.r, brake: out.brake,
            speed: control.effectiveSpeed(i),
            gl: Math.round(g.left), gr: Math.round(g.right),
            estop: i.estop,
            boost: i.boost,
            waitNeutral: false
        });
    }

    // Sensor telemetry to dashboard (up to 25 Hz)
    if (controller && now - lastSensorTelemAt >= SENSOR_TELEM_MS) {
        lastSensorTelemAt = now;
        send(controller, makeSensorPayload(now));
    }
}, TICK_MS);

if (require.main === module) {
    server.listen(PORT, HOST, () => {
        console.log(`Soccer & Maze Driver running. Open http://localhost:${PORT} in your browser.`);
        console.log(`Sending frames to the ESP32 at ${ESP32_IP}:${ESP32_PORT} (heartbeat ${TICK_MS} ms)`);
        console.log('Calibration: ' + calibText());
    });
}

function shutdown(code) {
    console.log('\nShutting down - sending STOP.');
    try { clearTimeout(saveTimer); fs.writeFileSync(CALIB_FILE, JSON.stringify(calib, null, 2)); } catch (_) {}
    sendStopBurst();
    setTimeout(() => process.exit(code), 120);
}
process.on('SIGINT', () => shutdown(0));
process.on('SIGTERM', () => shutdown(0));

module.exports = {
    createControlEngine,
    server,
    sensorState,
    calib,
    sendFrame,
    makeSensorPayload,
};
