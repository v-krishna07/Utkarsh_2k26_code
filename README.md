# Soccer Bot - ESP32 keyboard controller (UDP, minimum latency)

```
 Browser (dashboard.html)      Node.js (server.js + control.js)              ESP32 (soccer_esp32.ino)
 what the driver WANTS   --->  ALL the maths, sends a frame instantly  --->  applies a 6-byte frame
 keys, speed, turn, boost WS   on every key change + every 20 ms      UDP    to the L298N, failsafe
        <--- telemetry (what was actually sent) ---
```

The ESP32 does **no maths**. The browser does **no motor maths**. Everything is in `control.js`.

## Files

| File | What it does |
|---|---|
| `soccer_esp32.ino` | Flash this to the ESP32. Makes the Wi-Fi hotspot, receives frames, drives the L298N, brakes if the signal stops. |
| `server.js` | Runs on the laptop. Turns key presses into motor values and sends UDP frames. Serves the dashboard. |
| `control.js` | All calculation (speed, alpha trim, arcs, spin, boost, ramp, frame format). |
| `dashboard.html` | The control website (opened at http://localhost:8080). |
| `package.json` | Lists the one dependency (`ws`). |
| `calibration.json` | Created automatically when you change alpha. |

## 1. Hardware (same connections as your old tank)

Nothing changes from the old wiring. For reference:

| ESP32 pin | L298N pin | Job |
|---|---|---|
| GPIO 14 | ENA | Left side speed (PWM) |
| GPIO 26 | IN1 | Left direction |
| GPIO 27 | IN2 | Left direction |
| GPIO 12 | ENB | Right side speed (PWM) |
| GPIO 33 | IN3 | Right direction |
| GPIO 25 | IN4 | Right direction |
| GND | GND | **Must be joined** (battery, L298N and ESP32 share one ground) |

* Both left motors are wired in parallel to OUT1/OUT2, both right motors to OUT3/OUT4.
* The two small jumper caps on ENA and ENB must be **removed**, otherwise PWM speed control does nothing.
* If a side spins the wrong way, swap that side's two motor wires.
* GPIO 12 is a boot pin on the ESP32. Do not let ENB be pulled high while the ESP32 powers up (it works as you wired it).
* **12 V LiPo on 9 V motors:** full power is above the motor rating. If the motors get hot or die early, set `MAX_PWM: 191` in `control.js` (191/255 of 12 V is about 9 V).

## 2. First-time setup (do this at home, with internet)

1. Install the **ESP32 Arduino core 3.x** in the Arduino IDE. Open `soccer_esp32.ino`, pick your ESP32 board, upload.
2. Put `server.js`, `control.js`, `dashboard.html`, `package.json` in one folder.
3. In that folder run `npm install` **once, while you still have internet**. After you join the robot's Wi-Fi you will have no internet.

## 3. Every time you drive

1. Power the robot. Wait a few seconds. The Wi-Fi network **SoccerBot** (password `soccer12345`) appears.
2. Join it from the laptop. If the laptop says "no internet", choose to stay connected anyway.
3. In the folder run `node server.js`.
4. Open **http://localhost:8080** in Chrome or Edge. Keep **one** dashboard tab open (a second tab is refused on purpose so two tabs cannot fight over the motors).
5. First test with the robot **lifted on blocks**, so the wheels spin freely.

If the organisers assign you a Wi-Fi name or channel, edit `AP_SSID` / `AP_CHANNEL` at the top of `soccer_esp32.ino`, re-flash, and do not change it afterwards.

## 4. Controls

| Input | Action |
|---|---|
| `Up` / `Down` | Forward / backward |
| `Left` or `Right` alone | **Spin in place** |
| `Up` + `Right` (or any combo) | **Drive while turning** (arc). Left always makes the left wheel the inner one, also in reverse |
| `1` `2` `3` | Slow (100) / Normal (180) / Turbo (255) |
| Speed slider | Any speed 0-255. Slider and buttons share one speed: **whichever you touched last wins** |
| `[` `]` | Speed -10 / +10 |
| `B` (hold) | **Boost**: full 255 while held, then back to your speed |
| `Z` / `X` | Turn strength Normal (0.35) / Racing (0.75). Or type any value 0-1 |
| `Q` / `E` | **Goalkeeper** preset (speed 180, turn 0.5) / **Striker** preset (speed 255, turn 0.75). You can still change speed afterwards |
| `Enter` | Start / reset the 3:00 match clock |
| `Space` | **Emergency stop** (latching, brakes). After releasing, let go of all arrow keys once to re-arm |
| On-screen arrows | Same as the arrow keys (touch screens too) |

Other options on the page: **Smooth start** (about 0.1 s ramp, protects the battery from a current spike; stopping is always instant) and **Brake when keys are released** (motors are shorted so the robot stops dead, which helps a goalkeeper hold position).

Tip: some laptop keyboards cannot register three keys at once (for example `Up` + `Right` + `B`). If boost does not work while turning, that is the keyboard, not the code.

## 5. How the speed is calculated

```
1. Speed G (0..255)               buttons / slider / boost
2. Straight-line trim (alpha)     slower side = G, faster side = G * alpha
3. Turning (strength t)           inner wheel = corrected * (1 - 2t)
4. Dead-band, ramp, round
5. 6-byte frame to the ESP32
```

Example: G = 255, left faster, alpha 0.9 gives left 230, right 255, so it drives straight.

### Calibrating alpha (the motors never have exactly the same rpm)

1. Robot on the floor, speed Turbo, hold `Up`.
2. The robot pulls toward its **slower** side. So set **Faster side** to the *other* side.
3. Lower **alpha** (0.95, 0.9, 0.85 ...) until it drives straight. Check at Slow and Normal too.
4. It is saved on the laptop and reloaded next time.

## 6. Speed and heat: what was done

* **ESP32 does nothing but receive**: 80 MHz CPU, Wi-Fi sleep off, no sensors, no replies, no printing in the loop, 1 ms yield so it never busy-waits.
* **No waiting line**: the laptop sends a frame the instant a key changes, plus a heartbeat every 20 ms. If the socket ever backs up, old frames are dropped, not queued. The ESP32 empties its receive buffer every pass and applies only the **newest** valid frame.
* **UDP over the ESP32's own hotspot**: no router, no HTTP, no TCP retransmits.
* The server only listens on `localhost`, so nobody else on the robot's Wi-Fi can open the dashboard and drive.

## 7. Safety / failsafes

* ESP32 brakes if no valid frame arrives for **250 ms**.
* Server brakes if the browser goes quiet for **300 ms** (frozen tab, minimised window) and sends a burst of STOP when the dashboard closes or on Ctrl+C.
* Dashboard releases all keys when the window loses focus or the tab is hidden.
* Corrupted or foreign packets are ignored (header, checksum, direction and length are checked).
* Web pages from other sites cannot connect to the controller.

## 8. Frame format (UDP port 4210)

| Byte | Meaning |
|---|---|
| 0 | `0xA5` header |
| 1 | left direction: 0 coast, 1 forward, 2 reverse, 3 brake |
| 2 | left PWM 0..255 |
| 3 | right direction |
| 4 | right PWM 0..255 |
| 5 | XOR of bytes 0..4 |

## 9. Tuning

| What | Where |
|---|---|
| Preset speeds, turn presets, Goalkeeper / Striker presets, clock length | top of the `<script>` in `dashboard.html` |
| `RAMP_PER_SEC` (smooth start), `MIN_PWM` (motor dead-band: try 40-60 if motors hum at low PWM), `MAX_PWM`, `BOOST_SPEED`, `SPIN_SCALE` | `CONFIG` in `control.js` |
| Heartbeat rate (`TICK_MS`), timeouts, ESP32 IP / port, dashboard port | top of `server.js` |
| Wi-Fi name / password / channel, pins, PWM frequency, failsafe time, transmit power | top of `soccer_esp32.ino` |

## 10. Troubleshooting

| Problem | Fix |
|---|---|
| Dashboard says "Controller offline" | `node server.js` is not running, or you opened the HTML file directly. Use http://localhost:8080 |
| Dashboard says "Another dashboard is open" | Close the other tab. The page connects by itself once it is free |
| `Cannot find module 'ws'` | Run `npm install` in the folder (needs internet, do it before joining the robot's Wi-Fi) |
| `UDP send error` in the terminal | The laptop is not on the SoccerBot Wi-Fi |
| Robot does not move but dashboard is fine | Check you joined **SoccerBot**, the ESP32 Serial Monitor (115200) shows the hotspot, motors share ground with the ESP32, and the ENA/ENB jumpers are removed |
| Occasional lag | Turn off Wi-Fi power saving for the laptop's adapter; keep the laptop close; use the channel the organisers gave you |
| Motors hum but do not turn at low speed | Raise `MIN_PWM` in `control.js` |
| ESP32 resets when starting full speed | Leave **Smooth start** on; check the battery and the ESP32's 5 V supply |
| Pulls to one side | Calibrate alpha (section 5) |

## 11. Rulebook notes (from the PDF you gave)

* Max robot width is 12 cm including wheels. Length is free.
* The rulebook **bans** projectiles, launchers, springs under tension, flippers and grabbers, so there is deliberately **no kicker** in this code. "Kicking power" comes from Turbo + Boost and the instant brake for hard stops.
* The goalkeeper may not leave its penalty box and the striker may not enter its own. The robot has no sensors for this, so the driver has to keep track.
* Wi-Fi name and channel are assigned by the organisers: set them in the `.ino` (section 3).

## 12. What was tested

* `control.js`: 37,000+ checks (every key combination, both trim sides, reverse symmetry, boost, MAX_PWM cap, dead-band, ramp, frame round-trip, bad frames, garbage input).
* The server against a fake ESP32 over real UDP: 50 checks (heartbeat rate, a key press reaching the socket in about 1 ms, all drive modes, ramp, trim and saving, E-STOP and re-arm, silent-browser watchdog, STOP on close and Ctrl+C, second-dashboard refusal, wrong-origin and wrong-host refusal, oversized and garbage messages).
* The real dashboard in a real browser against the real server: 48 checks (keys, presets, slider vs buttons, roles, boost, trim, typing in boxes, E-STOP, window blur, on-screen pad, clock, auto-reconnect, no JavaScript errors).
* The real firmware compiled on a PC with stand-ins for the Arduino functions (no compiler warnings) and fed good and bad frames: 20 checks (pin levels for every case, newest-frame-wins, flood, failsafe, millis() wrap).
* **Not testable without your hardware:** the motors, the real ESP32 compile in the Arduino IDE, and Wi-Fi range. Use the robot on blocks for the first test.
