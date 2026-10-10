'use strict';
// ============================================================================
//  Soccer bot controller:  Browser (intent, WebSocket) -> Node.js (ALL math) -> ESP32 (UDP)
//
//  The browser only reports WHAT THE DRIVER WANTS (keys held, speed, turn strength,
//  boost, e-stop). This server turns that into left/right motor values (control.js)
//  and sends a 6-byte UDP frame to the ESP32:
//    - immediately whenever the driver's input changes (lowest latency), and
//    - every TICK_MS as a heartbeat / ramp update.
//  Only the newest state is ever sent. If the socket backs up, stale frames are
//  dropped instead of queued, so there is never a waiting line.
//
//  Usage:   node server.js                  (ESP32 at 192.168.4.1, its own hotspot)
//           node server.js 192.168.4.1      (override ESP32 IP)
//           PORT=9000 node server.js        (override dashboard port)
//  Then open http://localhost:8080
// ============================================================================
const http = require('http');
const fs = require('fs');
const net = require('net');
const path = require('path');
const dgram = require('dgram');
const WebSocket = require('ws');
const control = require('./control');

// ---------------------------- CONFIG ----------------------------------------
const ESP32_IP   = process.argv[2] || process.env.ESP32_IP || '192.168.4.1';
const ESP32_PORT = Number(process.env.ESP32_PORT) || 4210;
const PORT       = Number(process.env.PORT) || 8080;
const HOST       = process.env.HOST || '127.0.0.1';   // loopback only: nobody on the robot's Wi-Fi can drive it

const TICK_MS           = 20;    // heartbeat / ramp period (50 Hz). Input changes are sent immediately.
const INTENT_TIMEOUT_MS = 300;   // no fresh intent from the browser for this long -> brake
const TELEMETRY_MS      = 50;    // dashboard refresh (20 Hz)
const MAX_SEND_QUEUE    = 4;     // drop a frame instead of queueing more than this many
const CALIB_FILE = process.env.CALIB_FILE || path.join(__dirname, 'calibration.json');
// ----------------------------------------------------------------------------

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
function saveCalib() {
    fs.writeFile(CALIB_FILE, JSON.stringify(calib, null, 2), (err) => {
        if (err) console.error('Could not save calibration:', err.message);
    });
}
let calib = loadCalib();

// ---------------------------- UDP -------------------------------------------
const udp = dgram.createSocket('udp4');
udp.on('error', (err) => console.error('UDP socket error:', err.message));
udp.bind(0);                                          // bind now so the send queue can be inspected

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
    try { queued = udp.getSendQueueSize(); } catch (_) { /* socket not bound yet */ }
    if (queued > MAX_SEND_QUEUE) { framesDropped++; return; }   // never build a waiting line
    framesSent++;
    udpSend(control.toFrame(l, r, brake));
}

// UDP can lose packets, so send the stop several times.
function sendStopBurst() {
    const f = control.toFrame(0, 0, true);
    udpSend(f);
    setTimeout(() => udpSend(f), 20);
    setTimeout(() => udpSend(f), 40);
}

// ---------------------------- HTTP (dashboard) ------------------------------
const LOOPBACK = HOST === '127.0.0.1' || HOST === 'localhost' || HOST === '::1';

// Only answer requests addressed to localhost (blocks DNS-rebinding tricks).
function hostOk(req) {
    if (!LOOPBACK) return true;
    return /^(localhost|127\.0\.0\.1|\[::1\])(:\d+)?$/i.test(String(req.headers.host || ''));
}

// Any web page can open a WebSocket to localhost, so refuse pages from other sites.
// Allowed: no Origin header (non-browser tools) or the same host this server is reached at.
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
    server,
    maxPayload: 1024,
    verifyClient: (info) => hostOk(info.req) && originOk(info.req)
});

// ---------------------------- CONTROL STATE ---------------------------------
let controller = null;      // the one dashboard that is allowed to drive
let intent = null;          // last sanitized intent from the browser
let intentAt = 0;
let curL = 0, curR = 0;     // what is being sent to the motors (float while ramping)
let estopPrev = false;
let waitNeutral = false;    // after E-STOP release: stay stopped until all drive keys are released
let lastStepAt = Date.now();
let lastTelemetryAt = 0;
let lastOut = { l: 0, r: 0, brake: true };

function resetControlState() {
    intent = null;
    curL = 0; curR = 0;
    estopPrev = false;
    waitNeutral = false;
}

const DRIVE_KEYS = ['thr', 'str', 'speed', 'turn', 'boost', 'estop', 'smooth', 'brake'];
const sameIntent = (a, b) => a !== null && DRIVE_KEYS.every((k) => a[k] === b[k]);

function send(ws, obj) {
    if (ws && ws.readyState === WebSocket.OPEN && ws.bufferedAmount < 4096) ws.send(JSON.stringify(obj));
}
const configMsg = () => ({ type: 'config', alpha: calib.alpha, fasterSide: calib.fasterSide });

wss.on('connection', (ws) => {
    if (controller) {
        // A second dashboard would fight over the motors. First one wins.
        ws.close(4001, 'Another dashboard is already open');
        return;
    }
    controller = ws;
    resetControlState();
    console.log('>>> Dashboard connected.');
    send(ws, configMsg());

    ws.on('message', (raw) => {
        let msg;
        try {
            msg = JSON.parse(raw.toString());
        } catch (_) {
            return;                                   // not JSON: ignore
        }
        if (!msg || typeof msg !== 'object') return;

        if (msg.type === 'intent') {
            const next = control.sanitizeIntent(msg);
            const changed = !sameIntent(intent, next);
            intent = next;
            intentAt = Date.now();
            if (changed) step(intentAt);              // react NOW, do not wait for the next tick
        } else if (msg.type === 'calib') {
            calib = control.sanitizeCalib(msg, calib);
            saveCalib();
            send(ws, configMsg());
            step(Date.now());
            console.log(`Calibration: faster side = ${calib.fasterSide}, alpha = ${calib.alpha}`);
        }
    });

    ws.on('close', () => {
        if (controller !== ws) return;                // a rejected 2nd dashboard closing: ignore
        controller = null;
        resetControlState();
        console.log('>>> Dashboard disconnected - sending STOP.');
        sendStopBurst();
    });

    ws.on('error', (err) => console.error('WebSocket error:', err.message));
});

// ---------------------------- CONTROL STEP ----------------------------------
function step(now) {
    const dt = Math.min(0.1, Math.max(0, (now - lastStepAt) / 1000));
    lastStepAt = now;
    if (!controller) return;                          // idle: the ESP32 failsafe covers silence

    const fresh = intent !== null && (now - intentAt) <= INTENT_TIMEOUT_MS;
    const i = fresh ? intent : control.NEUTRAL;       // stale browser -> treat as "no input" (brake)

    // E-STOP release must not make the robot lunge if a key is still held.
    if (estopPrev && !i.estop) waitNeutral = true;
    estopPrev = i.estop;
    if (waitNeutral && i.thr === 0 && i.str === 0) waitNeutral = false;

    let target = [0, 0];
    let brake = true;                                 // E-STOP and the re-arm wait always brake
    if (!i.estop && !waitNeutral) {
        target = control.mix(i, calib).map((v) => control.applyDeadband(v));
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
    lastOut = { l, r, brake };
    sendFrame(l, r, brake);

    if (now - lastTelemetryAt >= TELEMETRY_MS) {
        lastTelemetryAt = now;
        const g = control.globalSides(control.effectiveSpeed(i), calib);
        send(controller, {
            type: 'telemetry',
            l, r, brake,
            speed: control.effectiveSpeed(i),
            gl: Math.round(g.left), gr: Math.round(g.right),
            estop: i.estop,
            boost: i.boost,
            waitNeutral
        });
    }
}
setInterval(() => step(Date.now()), TICK_MS);

server.listen(PORT, HOST, () => {
    console.log(`Soccer controller running.  Open  http://localhost:${PORT}  in your browser.`);
    console.log(`Sending frames to the ESP32 at ${ESP32_IP}:${ESP32_PORT}  (heartbeat ${TICK_MS} ms, instant on input change)`);
    console.log(`Calibration: faster side = ${calib.fasterSide}, alpha = ${calib.alpha}`);
});

// Brake the robot on Ctrl+C or a crash.
function shutdown(code) {
    console.log('\nShutting down - sending STOP.');
    sendStopBurst();
    setTimeout(() => process.exit(code), 120);
}
process.on('SIGINT', () => shutdown(0));
process.on('SIGTERM', () => shutdown(0));
process.on('uncaughtException', (err) => {
    console.error('Unexpected error:', err);
    shutdown(1);
});
