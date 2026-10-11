// ============================================================================
//  arm.h - 2-servo robotic arm driver for ESP32 (Core 3.x).
//
//  Servos:
//    LIFT: GPIO 13 (vertical elevation, down/up)
//    GRIP: GPIO 32 (upper rotating grip)
//
//  POWER WIRING:
//    Servo power must come from a SEPARATE 5 V supply (NOT the ESP32 3V3 pin),
//    sharing a COMMON GROUND with the ESP32.
//    Place a 470 µF to 1000 µF electrolytic capacitor across the 5 V and GND rails
//    close to the servos to buffer sudden current spikes and prevent brownout resets.
//
//  LEDC (Core 3.x):
//    50 Hz PWM frequency, 14-bit resolution (0..16383).
//    Duty count is calculated entirely on the laptop and transmitted via binary UDP.
//    The ESP32 does NO calculation.
//    Servos are NOT attached until the first valid arm frame arrives (prevents twitch).
//
//  Frame (UDP, port 4210), 6 bytes:
//      [0] 0xA6            header
//      [1] lift_duty_lo    low byte
//      [2] lift_duty_hi    high byte
//      [3] grip_duty_lo    low byte
//      [4] grip_duty_hi    high byte
//      [5] checksum        XOR of bytes 0..4
// ============================================================================
#ifndef ARM_H
#define ARM_H

#include <Arduino.h>

#define ARM_FRAME_HEADER  0xA6
#define ARM_FRAME_LEN     6

// Signal pins (checked: no clash with motors 14, 26, 27, 12, 33, 25 or sensors 16, 17, 18, 19, 22, 23)
#define SERVO_LIFT_PIN    13
#define SERVO_GRIP_PIN    32

#define ARM_PWM_FREQ      50
#define ARM_PWM_RES       14

// Duty limits corresponding to ~427 µs to ~2685 µs at 50 Hz, 14-bit (16384 ticks / 20000 µs)
#define ARM_DUTY_MIN      350
#define ARM_DUTY_MAX      2200

#ifndef ARM_DEBUG_SERIAL
#define ARM_DEBUG_SERIAL  0
#endif

static bool _armAttached = false;

// True if the arm frame is well formed and duties are within safe bounds.
static inline bool validArmFrame(const uint8_t* b) {
  if (b[0] != ARM_FRAME_HEADER) return false;
  uint8_t x = 0;
  for (int i = 0; i < ARM_FRAME_LEN - 1; i++) x ^= b[i];
  if (x != b[ARM_FRAME_LEN - 1]) return false;

  uint16_t liftDuty = (uint16_t)b[1] | ((uint16_t)b[2] << 8);
  uint16_t gripDuty = (uint16_t)b[3] | ((uint16_t)b[4] << 8);

  if (liftDuty < ARM_DUTY_MIN || liftDuty > ARM_DUTY_MAX) return false;
  if (gripDuty < ARM_DUTY_MIN || gripDuty > ARM_DUTY_MAX) return false;

  return true;
}

// Apply arm frame: lazily attach servos on first frame, then write duty.
static inline void armApplyFrame(const uint8_t* b) {
  if (!_armAttached) {
    bool ok1 = ledcAttach(SERVO_LIFT_PIN, ARM_PWM_FREQ, ARM_PWM_RES);
    bool ok2 = ledcAttach(SERVO_GRIP_PIN, ARM_PWM_FREQ, ARM_PWM_RES);
    if (!ok1 || !ok2) {
      Serial.printf("ERROR: ledcAttach failed for arm servos (lift=%d, grip=%d)!\n", (int)ok1, (int)ok2);
      return;
    }
    _armAttached = true;
  }

  uint16_t liftDuty = (uint16_t)b[1] | ((uint16_t)b[2] << 8);
  uint16_t gripDuty = (uint16_t)b[3] | ((uint16_t)b[4] << 8);

  ledcWrite(SERVO_LIFT_PIN, liftDuty);
  ledcWrite(SERVO_GRIP_PIN, gripDuty);

#if ARM_DEBUG_SERIAL
  Serial.printf("[ARM] Lift duty: %u, Grip duty: %u\n", liftDuty, gripDuty);
#endif
}

#endif // ARM_H
