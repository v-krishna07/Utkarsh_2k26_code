// ESP32 = hotspot + dumb sensor node. NO distance math here (raw echo microseconds only).
// Laptop joins WiFi "MazeBot", runs maze_test.js, which converts to cm and decides.
#include <WiFi.h>
#include <WiFiUdp.h>

const char*    AP_SSID = "MazeBot";
const char*    AP_PASS = "maze12345";      // min 8 chars
const uint16_t PORT    = 4210;

// Order: 0 = Left, 1 = Front, 2 = Right
// ECHO is 5 V on an HC-SR04 -> put a 1k + 2k divider before the ESP32 pin
const uint8_t TRIG[3] = {5, 18, 21};
const uint8_t ECHO[3] = {34, 35, 32};
const uint32_t TIMEOUT_US = 6000;          // ~100 cm; 0 = no echo = "far"

#define DEBUG_SERIAL 0                     // 1 = print raw values over USB for wiring checks

WiFiUDP udp;
IPAddress clientIP;
uint16_t clientPort = 0;
uint32_t lastHello = 0;
uint8_t seq = 0;

static inline uint16_t ping(uint8_t i) {
  digitalWrite(TRIG[i], HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG[i], LOW);
  return (uint16_t)pulseIn(ECHO[i], HIGH, TIMEOUT_US);
}

void setup() {
  setCpuFrequencyMhz(80);                  // cooler, still plenty for WiFi + 3 sensors
  Serial.begin(115200);
  for (uint8_t i = 0; i < 3; i++) {
    pinMode(TRIG[i], OUTPUT); digitalWrite(TRIG[i], LOW);
    pinMode(ECHO[i], INPUT);
  }
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS, 1, 0, 1);  // channel 1, visible, max 1 client
  WiFi.setTxPower(WIFI_POWER_11dBm);       // laptop is centimetres away, no need for full power
  udp.begin(PORT);
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());   // normally 192.168.4.1
}

void loop() {
  // laptop sends a tiny "hello" every 500 ms; that is how we learn its address
  if (udp.parsePacket() > 0) {
    clientIP = udp.remoteIP();
    clientPort = udp.remotePort();
    lastHello = millis();
    udp.flush();
  }
  // idle (no sensing, no sending) until the laptop is talking to us
  if (clientPort == 0 || millis() - lastHello > 3000) { delay(5); return; }

  uint16_t t[3];                           // sequential reads: no cross-talk between sensors
  t[0] = ping(0); delay(3);
  t[1] = ping(1); delay(3);
  t[2] = ping(2); delay(3);

  uint8_t buf[7];
  buf[0] = seq++;
  memcpy(buf + 1, t, 6);                   // little-endian uint16 x3
  udp.beginPacket(clientIP, clientPort);
  udp.write(buf, 7);
  udp.endPacket();

#if DEBUG_SERIAL
  Serial.printf("%u %u %u\n", t[0], t[1], t[2]);
#endif
}
