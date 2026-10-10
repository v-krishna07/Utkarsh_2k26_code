// ============================================================================
//  sensors.h - Non-blocking HC-SR04 driver for the maze mapper.
//
//  Three HC-SR04 sensors: LEFT, FRONT, RIGHT.
//  Fires one sensor at a time (L → F → R → L ...) with at least 2.5 ms gap
//  between triggers to avoid ultrasonic cross-talk.  7 ms timeout per sensor.
//
//  Uses echo-pin interrupts + a small state machine.  NEVER calls pulseIn().
//  Motor frames and the failsafe are never delayed.
//
//  Packet to the laptop (9 bytes, UDP, sent to the IP:port of the last motor frame):
//      [0] 0x5A            header
//      [1] seq             rolling sequence number (0..255)
//      [2] L_lo            left  echo microseconds, low  byte
//      [3] L_hi            left  echo microseconds, high byte  (0 = no echo)
//      [4] F_lo            front echo microseconds, low  byte
//      [5] F_hi            front echo microseconds, high byte
//      [6] R_lo            right echo microseconds, low  byte
//      [7] R_hi            right echo microseconds, high byte
//      [8] xor             XOR of bytes 0..7
//
//  WIRING:
//      Each HC-SR04 has 4 pins: VCC, TRIG, ECHO, GND.
//      VCC  -> 5 V rail (the sensor needs 5 V to work).
//      GND  -> common ground.
//      TRIG -> ESP32 GPIO directly (3.3 V is enough to trigger the sensor).
//      ECHO -> voltage divider 5 V → 3.3 V -> ESP32 GPIO.
//              Use a simple divider: 1 kΩ in series, then 2 kΩ to GND.
//              The junction goes to the ESP32 pin.  (5 V * 2/(1+2) = 3.33 V.)
//              WITHOUT this divider you WILL damage the ESP32 input.
//
//  PIN SELECTION:
//      Motor pins already used: 14, 26, 27, 12, 33, 25.
//      We pick GPIOs that are input-safe and not strapping/boot pins.
//      Change the six #defines below if your wiring differs.
// ============================================================================
#ifndef SENSORS_H
#define SENSORS_H

#include <WiFiUdp.h>

// ---- SENSOR PINS (change to match your wiring) ----------------------------
// Make sure these do NOT clash with motor pins: 14, 26, 27, 12, 33, 25
// Chosen: GPIOs that support input interrupts and are safe on ESP32-DevKit.
#define TRIG_LEFT   16
#define ECHO_LEFT   17
#define TRIG_FRONT  18
#define ECHO_FRONT  19
#define TRIG_RIGHT  22
#define ECHO_RIGHT  23
// Note: GPIO 16/17 on WROVER modules are used by PSRAM.  If you use a WROVER,
// change these to e.g. 2 and 4 (but GPIO 2 has an on-board LED on some boards).
// ----------------------------------------------------------------------------

#define SENSOR_COUNT      3
#define SENSOR_GAP_US     2500     // min 2.5 ms between triggers
#define SENSOR_TIMEOUT_US 7000     // 7 ms echo timeout (~120 cm)
#define SENSOR_TRIG_US    12       // trigger pulse width (spec: ≥10 µs)
#define SENSOR_PKT_HEADER 0x5A
#define SENSOR_PKT_LEN    9
#define SENSOR_IDLE_MS     2000    // don't send if no motor frame for this long

// ---- State machine ---------------------------------------------------------
enum SensorPhase {
    S_IDLE,         // waiting for the gap timer
    S_TRIG,         // trigger pulse is HIGH
    S_WAIT_ECHO,   // waiting for echo rising edge
    S_ECHO_HIGH     // echo is HIGH, measuring
};

static const int trigPins[SENSOR_COUNT] = { TRIG_LEFT, TRIG_FRONT, TRIG_RIGHT };
static const int echoPins[SENSOR_COUNT] = { ECHO_LEFT, ECHO_FRONT, ECHO_RIGHT };

static volatile uint32_t echoStart = 0;
static volatile uint32_t echoWidth = 0;
static volatile bool     echoDone  = false;

static volatile int activeEcho = ECHO_LEFT;

// ISR: fires on both edges of the active echo pin
static void IRAM_ATTR echoISR() {
    uint32_t now = micros();
    if (echoDone) return;
    if (digitalRead(activeEcho) == HIGH) {
        echoStart = now;
    } else if (echoStart > 0) {
        echoWidth = now - echoStart;
        echoDone = true;
    }
}

// Forward declarations for static helpers
static void _advanceSensor();
static void _sendSensorPacket(WiFiUDP& udpOut);

struct SensorState {
    SensorPhase phase;
    int         idx;            // 0 = L, 1 = F, 2 = R
    uint32_t    phaseStartUs;
    uint16_t    result[SENSOR_COUNT];   // latest echo µs per sensor (0 = no echo)
    uint8_t     seq;
    bool        roundDone;      // set after all 3 sensors have been read
    // Remote address (from last motor frame)
    IPAddress   remoteIP;
    uint16_t    remotePort;
    bool        hasRemote;
    uint32_t    lastMotorMs;    // millis() of last valid motor frame
};

static SensorState _sns;

void sensorsInit() {
    _sns.phase      = S_IDLE;
    _sns.idx        = 0;
    _sns.phaseStartUs = micros();
    _sns.seq        = 0;
    _sns.roundDone  = false;
    _sns.hasRemote  = false;
    _sns.lastMotorMs = 0;
    for (int i = 0; i < SENSOR_COUNT; i++) {
        _sns.result[i] = 0;
        pinMode(trigPins[i], OUTPUT);
        digitalWrite(trigPins[i], LOW);
        pinMode(echoPins[i], INPUT);
    }
}

// Call this whenever a valid motor frame is received, so we know where to reply.
void sensorsSetRemote(IPAddress ip, uint16_t port) {
    _sns.remoteIP   = ip;
    _sns.remotePort = port;
    _sns.hasRemote  = true;
    _sns.lastMotorMs = millis();
}

// Non-blocking tick.  Call this from loop() every iteration.
// udpOut: the WiFiUDP object to send sensor packets on.
void sensorsTick(WiFiUDP& udpOut) {
    uint32_t nowUs = micros();
    uint32_t elapsed = nowUs - _sns.phaseStartUs;

    switch (_sns.phase) {

    case S_IDLE:
        if (elapsed >= SENSOR_GAP_US) {
            // Start trigger pulse
            echoStart = 0;
            echoWidth = 0;
            echoDone  = false;
            digitalWrite(trigPins[_sns.idx], HIGH);
            _sns.phaseStartUs = nowUs;
            _sns.phase = S_TRIG;
        }
        break;

    case S_TRIG:
        if (elapsed >= SENSOR_TRIG_US) {
            digitalWrite(trigPins[_sns.idx], LOW);
            activeEcho = echoPins[_sns.idx];
            // Attach interrupt on this sensor's echo pin
            attachInterrupt(digitalPinToInterrupt(echoPins[_sns.idx]), echoISR, CHANGE);
            _sns.phaseStartUs = nowUs;
            _sns.phase = S_WAIT_ECHO;
        }
        break;

    case S_WAIT_ECHO:
        if (echoDone) {
            // Got a valid echo
            detachInterrupt(digitalPinToInterrupt(echoPins[_sns.idx]));
            _sns.result[_sns.idx] = (echoWidth > 0 && echoWidth <= SENSOR_TIMEOUT_US)
                                     ? (uint16_t)echoWidth : 0;
            _sns.phaseStartUs = nowUs;
            _sns.phase = S_IDLE;
            _advanceSensor();
        } else if (echoStart > 0) {
            // Echo started but not ended: still measuring
            _sns.phase = S_ECHO_HIGH;
            // Keep same phaseStartUs for timeout
        } else if (elapsed >= SENSOR_TIMEOUT_US) {
            // Timeout waiting for echo to start
            detachInterrupt(digitalPinToInterrupt(echoPins[_sns.idx]));
            _sns.result[_sns.idx] = 0;
            _sns.phaseStartUs = nowUs;
            _sns.phase = S_IDLE;
            _advanceSensor();
        }
        break;

    case S_ECHO_HIGH:
        if (echoDone) {
            detachInterrupt(digitalPinToInterrupt(echoPins[_sns.idx]));
            _sns.result[_sns.idx] = (echoWidth > 0 && echoWidth <= SENSOR_TIMEOUT_US)
                                     ? (uint16_t)echoWidth : 0;
            _sns.phaseStartUs = nowUs;
            _sns.phase = S_IDLE;
            _advanceSensor();
        } else if (elapsed >= SENSOR_TIMEOUT_US) {
            detachInterrupt(digitalPinToInterrupt(echoPins[_sns.idx]));
            _sns.result[_sns.idx] = 0;
            _sns.phaseStartUs = nowUs;
            _sns.phase = S_IDLE;
            _advanceSensor();
        }
        break;
    }

    // After a full round (L, F, R), send the packet
    if (_sns.roundDone) {
        _sns.roundDone = false;
        _sendSensorPacket(udpOut);
    }
}

static void _advanceSensor() {
    _sns.idx++;
    if (_sns.idx >= SENSOR_COUNT) {
        _sns.idx = 0;
        _sns.roundDone = true;
    }
}

static void _sendSensorPacket(WiFiUDP& udpOut) {
    // Don't send if we have no remote or the laptop went silent
    if (!_sns.hasRemote) return;
    if (millis() - _sns.lastMotorMs > SENSOR_IDLE_MS) return;

    uint8_t pkt[SENSOR_PKT_LEN];
    pkt[0] = SENSOR_PKT_HEADER;
    pkt[1] = _sns.seq++;
    pkt[2] = _sns.result[0] & 0xFF;         // L low
    pkt[3] = (_sns.result[0] >> 8) & 0xFF;   // L high
    pkt[4] = _sns.result[1] & 0xFF;         // F low
    pkt[5] = (_sns.result[1] >> 8) & 0xFF;   // F high
    pkt[6] = _sns.result[2] & 0xFF;         // R low
    pkt[7] = (_sns.result[2] >> 8) & 0xFF;   // R high
    uint8_t x = 0;
    for (int i = 0; i < SENSOR_PKT_LEN - 1; i++) x ^= pkt[i];
    pkt[8] = x;

    udpOut.beginPacket(_sns.remoteIP, _sns.remotePort);
    udpOut.write(pkt, SENSOR_PKT_LEN);
    udpOut.endPacket();
}

#endif // SENSORS_H
