// ============================================================================
//  ESP32 Soccer Bot - firmware. NO calculation here: the laptop does it all.
//
//  The ESP32 runs its own Wi-Fi hotspot (no router, lowest latency). The laptop joins it
//  and sends 6-byte UDP frames to 192.168.4.1:4210.
//
//  Frame (UDP, port 4210), 6 bytes:
//      [0] 0xA5            header
//      [1] left  dir       0 = coast, 1 = forward, 2 = reverse, 3 = brake
//      [2] left  pwm       0..255
//      [3] right dir       0 / 1 / 2 / 3
//      [4] right pwm       0..255
//      [5] checksum        XOR of bytes 0..4
//
//  L298N: each side is one channel (both motors of that side wired together).
//  Failsafe: motors brake if no valid frame arrives for TIMEOUT_MS.
//  Only the NEWEST valid frame in the receive queue is applied (no waiting line).
//  Requires ESP32 Arduino core 3.x (uses ledcAttach(pin, freq, resolution)).
// ============================================================================
#include <WiFi.h>
#include <WiFiUdp.h>
#include "sensors.h"     // maze mapper: non-blocking HC-SR04 driver
#include "arm.h"         // 2-servo robotic arm driver

// ---------------------------- CONFIG ----------------------------------------
// If the organisers assign you a Wi-Fi name / channel, put them here (and do not change them).
const char*   AP_SSID    = "SoccerBot";
const char*   AP_PASS    = "soccer12345";   // at least 8 characters
const uint8_t AP_CHANNEL = 1;
const uint16_t LOCAL_PORT = 4210;

// Left motors  (L298N channel A) - same pins as the old tank wiring
const int ENA = 14; const int IN1 = 26; const int IN2 = 27;
// Right motors (L298N channel B) - same pins as the ol d tank wiring
// (GPIO 12 is a boot-strap pin: it works with your driver as wired. Don't pull ENB HIGH at power-up.)
const int ENB = 12; const int IN3 = 33; const int IN4 = 25;

const int PWM_FREQ       = 5000;
const int PWM_RESOLUTION = 8;                    // 8 bit -> 0..255

const unsigned long TIMEOUT_MS = 250;            // no-frame failsafe
// ----------------------------------------------------------------------------

const uint8_t FRAME_HEADER = 0xA5;
const int     FRAME_LEN    = 6;
const uint8_t DIR_COAST = 0, DIR_FWD = 1, DIR_REV = 2, DIR_BRAKE = 3;

WiFiUDP udp;
uint8_t rxBuf[FRAME_LEN];

unsigned long lastPacketTime = 0;
bool failsafeActive = true;                      // motors are braked at boot

void driveSide(int inA, int inB, int pwmPin, uint8_t dir, uint8_t pwm) {
  if (dir == DIR_FWD && pwm > 0) {
    digitalWrite(inA, HIGH);
    digitalWrite(inB, LOW);
    ledcWrite(pwmPin, pwm);
  } else if (dir == DIR_REV && pwm > 0) {
    digitalWrite(inA, LOW);
    digitalWrite(inB, HIGH);
    ledcWrite(pwmPin, pwm);
  } else if (dir == DIR_BRAKE) {                 // both inputs high + enable on = motor shorted = brake
    digitalWrite(inA, HIGH);
    digitalWrite(inB, HIGH);
    ledcWrite(pwmPin, 255);
  } else {                                       // coast
    digitalWrite(inA, LOW);
    digitalWrite(inB, LOW);
    ledcWrite(pwmPin, 0);
  }
}

void brakeMotors() {
  driveSide(IN1, IN2, ENA, DIR_BRAKE, 255);
  driveSide(IN3, IN4, ENB, DIR_BRAKE, 255);
}

// True if the frame is well formed.
bool validFrame(const uint8_t* b) {
  if (b[0] != FRAME_HEADER) return false;
  uint8_t x = 0;
  for (int i = 0; i < FRAME_LEN - 1; i++) x ^= b[i];
  if (x != b[FRAME_LEN - 1]) return false;
  return b[1] <= DIR_BRAKE && b[3] <= DIR_BRAKE;
}

void applyFrame(const uint8_t* b) {
  driveSide(IN1, IN2, ENA, b[1], b[2]);
  driveSide(IN3, IN4, ENB, b[3], b[4]);
}

void setup() {
  setCpuFrequencyMhz(80);                        // lowest clock Wi-Fi allows: less heat
  Serial.begin(115200);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  ledcAttach(ENA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(ENB, PWM_FREQ, PWM_RESOLUTION);
  brakeMotors();
  sensorsInit();                                 // maze: set up HC-SR04 pins

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL, 0, 2);
  WiFi.setSleep(false);                          // no Wi-Fi power-save: steadier, lower latency
  WiFi.setTxPower(WIFI_POWER_15dBm);             // plenty for a small arena
  udp.begin(LOCAL_PORT);

  Serial.print("Hotspot '"); Serial.print(AP_SSID);
  Serial.print("' ready. IP: "); Serial.println(WiFi.softAPIP());
}

void loop() {
  // Drain everything that is waiting and keep only the NEWEST valid frame of each kind.
  bool gotMotor = false;
  bool gotArm = false;
  uint8_t newestMotor[FRAME_LEN] = {0};
  uint8_t newestArm[ARM_FRAME_LEN] = {0};
  IPAddress lastRemoteIP;
  uint16_t lastRemotePort = 0;
  bool gotAny = false;

  for (int guard = 0; guard < 32; guard++) {
    int n = udp.parsePacket();
    if (n <= 0) break;
    if (n == FRAME_LEN) {
      int len = udp.read(rxBuf, FRAME_LEN);
      if (len == FRAME_LEN) {
        if (rxBuf[0] == FRAME_HEADER && validFrame(rxBuf)) {
          memcpy(newestMotor, rxBuf, FRAME_LEN);
          gotMotor = true;
          lastRemoteIP = udp.remoteIP();
          lastRemotePort = udp.remotePort();
          gotAny = true;
        } else if (rxBuf[0] == ARM_FRAME_HEADER && validArmFrame(rxBuf)) {
          memcpy(newestArm, rxBuf, ARM_FRAME_LEN);
          gotArm = true;
          lastRemoteIP = udp.remoteIP();
          lastRemotePort = udp.remotePort();
          gotAny = true;
        }
      }
    }
    // any other size is not our frame and is ignored
  }

  // Any valid frame refreshes the sensor reply address
  if (gotAny) {
    sensorsSetRemote(lastRemoteIP, lastRemotePort);
  }

  // Motor frame and failsafe logic (strictly identical)
  if (gotMotor) {
    applyFrame(newestMotor);
    lastPacketTime = millis();
    failsafeActive = false;
  } else if (!failsafeActive && millis() - lastPacketTime > TIMEOUT_MS) {
    brakeMotors();
    failsafeActive = true;
  }

  // Arm frame logic (holds position when idle, no failsafe)
  if (gotArm) {
    armApplyFrame(newestArm);
  }

  sensorsTick(udp);                               // maze: non-blocking sensor scan
  delay(1);                                      // yield: no busy-wait, so the chip stays cool
}
