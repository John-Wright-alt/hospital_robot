# Phase 8 — Face-Tracking Head Column (wiring rev 2)
### Complete step-by-step guide · ESP32-S3 camera on the column · lift motor on the Pico · ROS 2 Jazzy on the Pi

**Where you are:** Phases 0–6 are done or nearly done. Stage A of the first version of this guide (camera + face detection on the ESP32) is done.
**What changed:** your wiring diagram (rev 2) puts the lift motor, its driver and the home switch in the robot body, wired to the **Pico**. Only the FireBeetle goes on the column, with a single USB cable. This guide replaces the first GUIDE.md completely. Follow this one from now on.

| Where you are | Start at |
|---|---|
| You did Stage A with the first guide | **Part 5.5** (it takes 10 minutes), then Part 6 |
| Starting from scratch | Part 1 |

---

## 0. Read this first

### What you are building

```
 Robot armed (a person arms it - never automatic)
       │
       ▼
 HOMING ── the Pico drives the lift DOWN until it presses the micro switch  (now it knows 0 mm)
       │
       ▼
 SEARCHING ── the lift rises slowly while the camera on top looks for a face
       │  face seen
       ▼
 TRACKING ── the lift moves up/down to keep the patient's face in the middle of the picture
       │  patient types a destination on the touchscreen
       ▼
 PARKING ── the lift drives down onto the micro switch
       │
       ▼
 PARKED ── /head_column/parked = true  ──►  Nav2 drives the robot to the destination
       │  robot arrives, the touchscreen code sends "search"
       └──────────────► back to SEARCHING
```

### Who does what (wiring rev 2)

```
   COLUMN (moves)                          BODY
 ┌──────────────────┐   one USB-C cable   ┌──────────────────────────────────────────────────────┐
 │ FireBeetle       │ ──────────────────► │ Raspberry Pi 4                                        │
 │ ESP32-S3 +camera │   'f' face lines    │   head_column node ── /lift/command ──► pico_bridge   │
 └──────────────────┘   (Pi USB port 2)   │      ▲  /head_column/park (service)          │ USB 1  │
                                          │      │                                       ▼        │
                                          │ Touchscreen app                    Pico W (lift.h)    │
                                          │   → park → Nav2 → search           GP10-13 → DRV8825  │
                                          │                                    → NEMA 17 + switch │
                                          └──────────────────────────────────────────────────────┘
```

| Board | Where | Job |
|---|---|---|
| **FireBeetle ESP32-S3** | top of the column | Camera + face detection only. Sends one line per camera frame to the Pi over USB |
| **Pico W** | body | Moves the lift: step pulses, home switch, homing, parking, limits (new file `lift.h`) |
| **Raspberry Pi** | body | Decides: search / track / park (new ROS node `head_column`). Talks to the Pico through your existing `pico_bridge` |

**Only one cable goes up the column: the ESP32's USB-C cable.** The robot doesn't need WiFi for any of this. WiFi stays available on the ESP32 for a browser preview while you test.

The lift's motor power comes from your **switched motor bus** (after the E-stop and relay). So the lift can only move while the Pico is **ARMED**, and the E-stop stops the lift as well as the wheels.

### A realistic note about the Instagram video

The post's "zero delay" and "0-pixel accuracy" are marketing. Every detector has some pixel error, and every frame takes time to process. On this chip, expect **a few frames per second**. This system is designed around that honestly:
- it measures the error on each frame,
- it ignores small errors (a "deadband"),
- it moves slowly.

That is exactly the behaviour you want next to a patient's face. It also only *detects* faces. It does not recognise who anyone is, and no pictures are saved. That is worth saying in your report (patient privacy).

### The files

Everything is in **`HeadColumnProject_rev2.zip`**:

```
HeadColumnProject_rev2/
├── GUIDE.md                          ← this file
├── firmware/                         ← ESP32 (Arduino IDE)
│   ├── BoardCheck/BoardCheck.ino     ← first-upload test
│   └── HeadColumn/                   ← ESP32 firmware: open HeadColumn.ino
│       ├── HeadColumn.ino, config.h, vision.h, web.h
│       └── motion.h                  ← no longer used with rev 2 (leave it there)
├── pico_firmware/
│   └── lift.h                        ← goes INTO your pico_motor_firmware folder (Part 7)
├── ros/                              ← changes to your Phase 6 ROS package (Part 9)
│   ├── phase8_ros.diff               ← all changes as one patch
│   └── hospital_robot_base/          ← the same changes as complete files
│       ├── hospital_robot_base/head_column_node.py   NEW: search / track / park
│       ├── hospital_robot_base/pico_bridge.py        + lift commands and status
│       ├── hospital_robot_base/pico_protocol.py      + reading 'l' and 'f' lines
│       ├── config/base.yaml, launch/base.launch.py, setup.py
│       └── test/test_pico_protocol.py                + 4 new tests
└── pi/                               ← small tools for the Pi
    ├── lift_console.py               ← test the lift without ROS (Part 7)
    ├── park_then_navigate_example.py ← touchscreen → park → Nav2 → search example
    └── esp_test.py                   ← optional: reads the ESP32's WiFi preview API
```

You can delete the old `HeadColumnProject` folder once you've moved to this one. Its `head_column_bridge.py` and `fake_esp.py` are no longer used.

### The plan (do the stages in order — each one proves something works before you add more)

| Stage | What | Needs |
|---|---|---|
| Part 2–4 | Install software, first upload | FireBeetle + USB-C cable |
| **Stage A** (Part 5) | Camera + face detection, face data over USB | FireBeetle + camera |
| **Stage B** (Part 6–7) | Lift driver + switch on the Pico, on the bench | Your Phase 6 bench setup + DRV8825, motor, switch |
| **Stage C** (Part 8) | Lift on the real column: direction, travel, speed, force | The column |
| **Stage D** (Part 9) | ROS 2: the full search / track / park behaviour | Pi + Pico + ESP32 |

> **Finish Phase 6 Stage B (Part 8, drive base on the bench) before Stage B here.** The drive base should work first. You can wire the DRV8825 during your Phase 6 bench build: its 10 kΩ pull-up keeps it switched off until the new firmware exists.

### How to read this guide

- **"On the PC"** = a normal terminal on your Ubuntu computer.
- **"On the Pi"** = a terminal SSH'd into the Pi (`ssh wright@robot.local`), or VS Code's terminal connected with Remote-SSH.
- Run command blocks **one at a time** and read the "you should see" line before moving on.
- **Commit to git after every Part that works.**

---

## 1. Things you need

| Item | Notes (wiring diagram rev 2, page 3) |
|---|---|
| DFRobot FireBeetle 2 ESP32-S3 with OV2640 camera | On the column |
| **USB-C data cable**, long enough for the full lift travel + a slack loop | The only cable up the column. Many cables are *charge-only* and won't work |
| DRV8825 carrier (U3) | In the body, driven by the Pico |
| NEMA 17 42BYGHW208 (M3) | 12 V, 0.4 A per phase, about **30 Ω per coil** |
| C3 — 470 µF / 25 V electrolytic capacitor | Across VMOT–GND right at the driver. Mandatory |
| R16 — 10 kΩ resistor | From EN to 3V3: keeps the driver off until the firmware turns it on |
| KW11-2 snap-action micro switch, 50 mm lever (SQ1) | Bottom of the lift travel |
| Multimeter, small screwdriver (plastic/ceramic is best) | For Vref and the coil check |
| Your Phase 6 bench setup | Pico, battery, E-stop, relay: the lift is powered from the switched motor bus |
| Optional, recommended later: a 2nd micro switch at the top | Adds a hardware top limit |

---

## 2. Set up the Arduino IDE on your Linux computer

> Already done if you completed Stage A. Skip to Part 5.5.

### 2.1 Make the Arduino IDE start

On the PC, open a **Terminal** (Ctrl + Alt + T) and type:

```bash
chmod +x ~/Downloads/arduino-ide_*.AppImage
~/Downloads/arduino-ide_*.AppImage --no-sandbox
```

The first line gives the file permission to run; the second starts it. Keep that terminal open while you use the IDE.

> **If you get an error mentioning `libfuse.so.2`:**
> Ubuntu 22.04: `sudo apt install libfuse2` · Ubuntu 24.04: `sudo apt install libfuse2t64` — then try again.

### 2.2 Give yourself permission to use the USB port (one time only)

```bash
sudo usermod -a -G dialout $USER
```

Then **log out and log back in**. Check with `groups`: the list should include `dialout`.

### 2.3 Install the ESP32 board package — **version 2.0.17 exactly**

> **Why an old version?** The face detector (Espressif "esp-dl") is built into ESP32 package version 2.0.x and was removed in 3.x. With 3.x installed, the firmware refuses to compile and tells you so.

1. **File → Preferences**. In **"Additional boards manager URLs"** paste:
   ```
   https://espressif.github.io/arduino-esp32/package_esp32_index.json
   ```
   Click **OK**. (If the Pico's URL is already there from Phase 4, put this one on a new line — both can be listed.)
2. **Tools → Board → Boards Manager…**, search `esp32`.
3. On **"esp32 by Espressif Systems"**, choose **`2.0.17`** in the version drop-down and click **INSTALL**. Wait until it finishes.
4. If the IDE later offers "updates available" for esp32, click **Later**. Don't update it.

### 2.4 Libraries

**None needed.** Everything the ESP32 firmware uses is inside the ESP32 package.

---

## 3. ESP32 board settings (Tools menu)

Plug the FireBeetle into the PC. Set **every** line below in the **Tools** menu. Four of them are *not* the defaults.

| Tools menu item | Set it to | |
|---|---|---|
| Board | **ESP32S3 Dev Module** (Tools → Board → esp32) | |
| USB CDC On Boot | **Enabled** | ⚠️ not default |
| Flash Size | **16MB (128Mb)** | ⚠️ not default |
| Partition Scheme | **Huge APP (3MB No OTA/1MB SPIFFS)** | ⚠️ not default — firmware is ~1.5 MB |
| PSRAM | **OPI PSRAM** | ⚠️ not default — the camera needs it |
| Flash Mode | QIO 80MHz | default |
| USB Mode | Hardware CDC and JTAG | default |
| Upload Mode | UART0 / Hardware CDC | default |
| Upload Speed | 921600 | default |
| Port | `/dev/ttyACM0` (or ttyACM1…) | |

> ⚠️ **The Arduino IDE remembers the last board.** When you switch between the ESP32 and the Pico (Part 7), always check **Tools → Board** and **Tools → Port** before uploading.

---

## 4. First upload — BoardCheck

> Already done if you completed Stage A.

### 4.1 Put the project in your Arduino folder

On the PC, with the zip in your Downloads folder:

```bash
mkdir -p ~/Arduino
cd ~/Arduino
unzip ~/Downloads/HeadColumnProject_rev2.zip
ls ~/Arduino/HeadColumnProject_rev2
```

You should see `GUIDE.md  firmware  pi  pico_firmware  ros`.

### 4.2 Upload BoardCheck

1. `ls /dev/ttyACM*` should show `/dev/ttyACM0`. If not, try another USB-C cable.
2. **File → Open…** → `~/Arduino/HeadColumnProject_rev2/firmware/BoardCheck/BoardCheck.ino`.
3. Check **Tools → Board** (ESP32S3 Dev Module) and **Tools → Port**, then click **→ (Upload)**.
4. Success ends with **"Hard resetting via RTS pin…"**.

> **If upload fails ("Failed to connect"):** hold **BOOT**, tap **RST**, release **BOOT**, click Upload again, then press **RST** afterwards.

### 4.3 Check the result

Open the **Serial Monitor** (magnifying glass, top right). Set **115200 baud** and **New Line**. Every second you should see:

```
PSRAM: 8 MB   [OK]
Arduino ESP32 core version: 2.0.17  [OK]
```

---

## 5. Stage A — Camera and face detection

> Did Stage A with the first guide? Do only **5.1** (if the camera is off) and **5.5**.

### 5.1 Connect the camera

With the USB cable **unplugged**: flip up the black latch of the **CAM** connector, slide the ribbon in fully with **pin 1 at the white dot** printed on the board, and press the latch closed. Never force it.

### 5.2 Open the firmware and set WiFi (for the browser preview)

1. **File → Open…** → `~/Arduino/HeadColumnProject_rev2/firmware/HeadColumn/HeadColumn.ino`.
   Tabs open: `HeadColumn.ino`, `config.h`, `motion.h`, `vision.h`, `web.h`.
2. Click the **`config.h`** tab. It's the only ESP32 file you edit.
3. Set your WiFi (keep the quote marks):
   ```cpp
   #define WIFI_SSID          "YOUR_WIFI_NAME"
   #define WIFI_PASSWORD      "YOUR_WIFI_PASSWORD"
   ```
   ⚠️ **It must be the 2.4 GHz network.** A name ending in `_5G` (like `Vodacom-V6WF_5G`) is 5 GHz, and the ESP32 can't see it. Use the same name without `_5G`. Check with `nmcli -f SSID,FREQ dev wifi list`: 2.4 GHz networks show 2412–2472 MHz.
4. Leave these as they are:
   ```cpp
   #define ENABLE_CAMERA   1
   #define ENABLE_MOTOR    0     // ALWAYS 0 with wiring rev 2 - the Pico drives the lift
   #define ENABLE_WIFI     1     // browser preview; set 0 on the finished robot
   ```

### 5.3 Upload and find the address

Upload (check Board and Port first). In the Serial Monitor (115200) you'll see:

```
[camera] running, face detection active
 WiFi connected!  Open  http://192.168.1.50   (or http://headcolumn.local)
Camera-only mode: the Pico drives the lift (wiring rev 2).
```

> If WiFi fails, the board makes its own network **HeadColumn-Setup** (password `headcolumn123`). Connect your laptop to it and open **http://192.168.4.1**.

### 5.4 Watch it detect your face

Open the address in a browser. You'll see:
- the live picture,
- a **dashed cyan line**: where the face should be,
- a **green box** around your face,
- a **yellow line**: the error the lift will correct.

(The page's Search/Park buttons do nothing with rev 2. The lift is controlled from the Pi.)

In the Serial Monitor type `status`:

```
Camera: 6.8 fps | Face: YES centre y=86, width=71 px -> error +71.8 mm (column should go UP)
```

**Key check:** move your head **above** the cyan line → it must say **"column should go UP"**; below → **"DOWN"**. If it's reversed, the camera is upside down: set `#define CAMERA_VFLIP 1` and upload again.

Also note:
- **Frames per second:** a few is normal. Write the number down for your report.
- **Face width at your furthest distance:** set `MIN_FACE_WIDTH_PX` a little below it, so people in the background are ignored.

### 5.5 Face data over USB (new with rev 2)

If you did Stage A with the first guide: open the new `HeadColumn.ino` from `HeadColumnProject_rev2` (5.2), copy your WiFi name, password and any `CAMERA_VFLIP` / `MIN_FACE_WIDTH_PX` change into the new `config.h`, and upload.

In the Serial Monitor type `faces on`. One line appears per camera frame:

```
f 812 1 160 86 71 71.8 6.8
  │   │  │  │  │   │    └ frames per second
  │   │  │  │  │   └ error in mm (+ = the lift should go UP)
  │   │  │  │  └ face width in pixels
  │   │  └──┴ face centre x, y (pixels)
  │   └ 1 = face found, 0 = no face
  └ frame number
```

Move your head up and down: the error changes sign. Type `faces off` to stop. On the robot, the Pi switches this on by itself.

✅ **Stage A done** when the green box follows your face, UP/DOWN is correct, and `faces on` gives `f` lines.

---

## 6. Wiring the lift to the Pico (wiring diagram rev 2, page 3)

> ⚠️ **Safety rules for this whole Part**
> - Wire everything with **all power off**: battery disconnected, USB unplugged.
> - **Never plug or unplug the stepper while VMOT is live.** Powering the driver with the motor unplugged is fine; *unplugging* it while powered destroys the DRV8825.
> - All grounds go to the star GND bus (diagram page 1).

### 6.1 What goes where

- **In the body:** Pico W, DRV8825, NEMA 17, the KW11-2 home switch at the bottom of the travel, C3 and R16.
- **On the column:** only the FireBeetle + camera, connected by **one USB-C cable to the Pi's USB port 2** (diagram page 5; the Pico is on USB port 1).

### 6.2 Pico → DRV8825

| Pico pin | Signal | DRV8825 pin | Notes |
|---|---|---|---|
| 14 (GP10) | STEP | 7 STEP | |
| 15 (GP11) | DIR | 8 DIR | |
| 16 (GP12) | EN | 1 EN | plus **R16 10 k from EN to 3V3** (driver OFF by default) |
| 36 (3V3 OUT) | 3.3 V | 5 RESET + 6 SLEEP | bridge pins 5 and 6 with a solder blob, one wire to 3V3 |
| 36 (3V3 OUT) | 3.3 V | 2 M0 and 3 M1 | 1/8 microstepping |
| GND (pin 13 or 18) | GND | 4 M2 and 9 GND (logic) | M2 low → 1/8 microstepping |

Motor side:

| From | To DRV8825 |
|---|---|
| VMOT, switched motor bus (orange 20 AWG, page 1) | 16 VMOT |
| Star GND (black 20 AWG) | 15 GND (motor) |
| C3 470 µF / 25 V: + leg to VMOT, stripe (−) to GND | right at the driver pins |
| Motor coil A (A+ / A−) | 12 A1 / 11 A2 |
| Motor coil B (B+ / B−) | 13 B1 / 14 B2 |
| — | 10 FAULT: not connected |

**Finding the coil pairs:** multimeter on Ω. Two wires reading about **30 Ω** are one coil; wires from different coils read open. It's often Black/Green = A and Red/Blue = B, but measure. A reversed coil makes the motor buzz without turning.

### 6.3 The home switch (fail-safe NC wiring)

| KW11-2 leg | Pico |
|---|---|
| C (common) | pin 18 GND (black 24 AWG) |
| NC | pin 17 GP13 (brown 24 AWG) |
| NO | not used |

How it reads (the firmware switches on the Pico's internal pull-up):

| Situation | GP13 | Meaning |
|---|---|---|
| Lever free | LOW | lift is off the switch |
| Lever pressed | **HIGH** | lift is at the bottom (0 mm) |
| Wire broken | HIGH | looks "pressed", so the lift refuses to go further down. **Fail-safe** |

> ⚠️ **Correct your wiring diagram, page 3, "HOMING SWITCH LOGIC" note.** It says LOW = at home. That's backwards: pressing a snap-action switch *opens* its NC contact, so at home GP13 reads **HIGH**. The wiring itself is right. Change the note to "HIGH = pressed = at home · LOW = released = off home", and the boot routine to "step down until GP13 reads HIGH". Also on page 3: this motor's coils read about 30 Ω, not 2–5 Ω.

Twist the two switch wires together and keep them away from the motor wires.

### 6.4 The ESP32 on the column

The FireBeetle's USB-C goes to the **Pi's USB port 2**. That one cable powers it and carries the face data. Nothing else connects to it.

The head moves, so the cable must too:
- Leave a **slack loop** (or use a small cable chain) long enough for the full travel plus about 5 cm.
- Clip it so it can't catch in the lead screw.

### 6.5 Set the DRV8825 current limit (Vref)

Formula: **current limit = Vref × 2**. Your diagram says **Vref = 0.20 V** (0.4 A, the motor's rating). The factory setting is 1–2 A and would cook this motor within a minute.

The DRV8825 makes its own logic supply from **VMOT**, and VMOT is only live while the Pico is **ARMED**. So you need firmware 1.0.1 (or newer) on the Pico:

1. Power off. **Unplug the stepper** from the driver.
2. Power up the robot bench setup and the Pi. On the Pi, arm the Pico from your Phase 6 tools (`pico_console.py`, or miniterm on `/dev/pico` and type `a 1`). The relay clicks and VMOT is live.
3. Multimeter on DC volts. Black probe on GND, red probe on the metal body of the small trimmer pot. Turn it in tiny steps until it reads **0.20 V**.
4. Disarm (`a 0`), **power everything off**, reconnect the motor.

> **Why the exact Vref matters less than you'd think:** from an 11.1 V battery, a ~30 Ω coil can only draw about 0.37–0.42 A anyway. So the motor's own resistance already limits the current near 0.4 A. That's good for safety: this small motor (2.6 kg·cm) can't push hard. If you want it to stall *earlier* (Part 8.4), you need Vref **below about 0.18 V**.

✅ **Part 6 done** when everything is wired, the coils are checked, and Vref is 0.20 V.

---

## 7. Stage B — The lift on the bench (Pico firmware 1.1.0)

The motor can lie on the table, not attached to anything. Put a tape flag on its shaft so you can count turns.

### 7.1 Check the settings at the top of lift.h

On the PC, open `~/Arduino/HeadColumnProject_rev2/pico_firmware/lift.h` in a text editor. The **SETTINGS** block at the top holds everything you might change:

```cpp
#define LIFT_DIR_HIGH_MEANS_UP   1     // lift goes the wrong way? change to 0
#define LIFT_MICROSTEPS          8     // M0 + M1 to 3V3, M2 to GND (page 3)
#define LIFT_MM_PER_REV          8.0f  // lead screw lead (T8x8 = 8, T8x2 = 2). MEASURE
#define LIFT_TRAVEL_MM           400.0f  // switch -> mechanical top. MEASURE
#define LIFT_MAX_SPEED_MM_S      20.0f
```

`LIFT_MM_PER_REV` is how far the nut moves when the screw turns once. If unsure, mark the screw, turn it once by hand, and measure. Leave the rest for now; you measure the travel in Part 8.

### 7.2 Add lift.h to your Pico firmware

Your Pico firmware is the `pico_motor_firmware` sketch from Phases 4 and 6 (version 1.0.1). You make **one new file** and **seven small edits**.

**1. Copy the file.** Copy `lift.h` into your `pico_motor_firmware` sketch folder, next to `pico_motor_firmware.ino` and `config.h`. On the PC, for example (use your real folder path):

```bash
cp ~/Arduino/HeadColumnProject_rev2/pico_firmware/lift.h ~/Arduino/pico_motor_firmware/
```

Close and reopen the sketch in the Arduino IDE. A new **`lift.h`** tab appears.

**2. `config.h` tab:** change the version line to:

```cpp
#define FIRMWARE_VERSION "1.1.0"   // 1.1.0: head-column lift (lift.h, Phase 8)
```

**3. `pico_motor_firmware.ino`, at the very top**, where the `#include` lines are. Add this line **below** `#include "config.h"`:

```cpp
#include "lift.h"
```

**4. In `setup()`**, right after the `Serial.begin(...)` line:

```cpp
  liftBegin();
```

**5. In `loop()`**, right after the line `pollSerial(now);`:

```cpp
  liftUpdate(armState == ARMED);
```

**6. In your serial command handler**: the code that recognises the letters `v`, `a`, `c`, `i`, `k`. Add the letter `l`. It must pass the text **after** the `l`, and whether the robot is armed. Use the same variable your `v` handler reads its numbers from. If the handler is a `switch` on the first character of a text buffer called `line`:

```cpp
    case 'l': liftCommand(line + 1, armState == ARMED); break;
```

or, if it's a chain of `if`s:

```cpp
    else if (line[0] == 'l') liftCommand(line + 1, armState == ARMED);
```

**7. In `driversOffNow()`** (the function the E-stop calls), add as its first line:

```cpp
  liftEmergencyStop();
```

That makes the lift stop in the same instant as the wheels.

**8. Optional but useful:**
- In the `i` handler, add `liftPrintInfo();`.
- In the `h` help text, add ` | l h|p|g|s|?`.

> **Only if your `say()` function uses a lock** (because both Pico cores print), add this line **above** `#include "lift.h"`:
> ```cpp
> #define LIFT_PRINTLN(text) say("%s", text)
> ```

**9. Compile and upload:**
1. **Tools → Board → Raspberry Pi Pico W**, and check the Port.
2. Click **Verify** (✓). You should see "Done compiling".
3. Upload exactly as in Phase 6 Part 8.2.
4. Commit the firmware to git.

> If Verify fails with an error mentioning `liftCommand`, `line` or `armState`, the names in steps 5–7 differ from your firmware's. Look at how your `v` handler gets its numbers, and use the same names.

### 7.3 Copy the project to the Pi (one time)

On the PC:

```bash
cd ~/Arduino
scp -r HeadColumnProject_rev2 wright@robot.local:~/
```

On the Pi, `ls ~/HeadColumnProject_rev2` should list the same folders.

### 7.4 Test the lift from the Pi, without ROS

Your Pico prints `e` encoder lines 50 times a second, which buries everything in miniterm. `lift_console.py` hides them and shows only messages, the arm state and the lift status.

**Stop `pico_bridge` first** if it's running (Ctrl+C in its terminal). Only one program can use `/dev/pico` at a time. Then on the Pi:

```bash
python3 ~/HeadColumnProject_rev2/pi/lift_console.py
```

Type each command and press Enter. Check the result before moving on:

| Type | You should see |
|---|---|
| `l ?` | `switch GP13 reads LOW -> released` |
| (hold the switch pressed) `l ?` | `reads HIGH -> PRESSED` |
| (unplug the GP13 wire) `l ?` | `PRESSED`: the fail-safe works. **Plug it back in** |
| `l h` | `# lift: not armed - send 'a 1' first`: the lift can't move without motor power ✅ |
| `a 1` | `[arm] ARMED` |
| `l h` | The shaft turns slowly (this is "down"). **Tap** the switch (press and release): it stops, turns back a little, then creeps. **Tap again within about 5 s** → `# lift: homed. switch = 0 mm` |
| `l g 80` | With an 8 mm lead the shaft turns **exactly 10 times**. Count the flag. If not, fix `LIFT_MM_PER_REV` or `LIFT_MICROSTEPS` and upload again |
| `l g 20` | Turns back 7.5 times |
| `l p` | Runs "down" to 10 mm, then creeps. Tap the switch → `# lift: parked. position error corrected: 0.0 mm` |
| `l g 300`, then press the **E-stop** while it turns | Stops instantly: `motor power off - lift position forgotten` |
| release the E-stop, `a 1`, `l h` | Homes again (tap twice). After any power loss the lift must re-home — by design |
| `quit` | Stops the lift and disarms |

If you don't tap the switch during homing, it gives `FAULT 1: switch never triggered`. That's the safety check working; type `l h` to retry.

What the `[lift]` line means:

```
[lift] MOVING   homed=1 pos=   42.5 mm  target=   80.0 mm  switch=free
```

The raw line from the Pico behind it is `l <state> <homed> <pos_mm> <target_mm> <switch> <fault>`. States: 0 IDLE, 1 MOVING, 2 HOMING, 3 PARKING, 4 FAULT.

✅ **Stage B done** when the switch reads correctly, homing works, distances are right, and the E-stop stops the lift.

---

## 8. Stage C — On the real column

### 8.1 Mount everything

1. Mount the FireBeetle and camera on the head, **camera upright** (or set `CAMERA_VFLIP` to match), pointing **straight ahead**.
2. Route the USB cable with its **slack loop** (6.4).
3. Move the head through the full travel slowly (by hand if the mechanism allows) and check the cable can't snag.

### 8.2 First movement — be ready to stop it

Have a hand on the **E-stop**. On the Pi:

```bash
python3 ~/HeadColumnProject_rev2/pi/lift_console.py
```

Type `a 1`, then `l h`. **The lift must go DOWN.**

If it goes **up**:
1. Press the E-stop.
2. In `lift.h` set `#define LIFT_DIR_HIGH_MEANS_UP 0`.
3. Upload the Pico firmware again and repeat.

Let it find the switch (on the column the carriage presses it by itself). The lift is now at 0 mm.

### 8.3 Measure your column

1. Type `l g 100`. Measure with a ruler how far the head moved. It must be 100 mm; if not, recheck `LIFT_MM_PER_REV`.
2. Go up in steps (`l g 200`, `l g 300`…) until you're close to the top. **Stop before the mechanical end.**
3. Put the real numbers into **`lift.h`**:
   ```cpp
   #define LIFT_TRAVEL_MM    400.0f   // real travel from the switch to the mechanical top
   #define LIFT_SOFT_MAX_MM  390.0f   // about 10 mm less
   ```
4. Upload the Pico firmware again. Also remember the soft maximum: you'll put the same number in `base.yaml` in Part 9.3.

### 8.4 Tune speed and force (the "stall rather than crush" test)

1. With the head fully assembled, try `l g 300` and `l p` a few times.
2. **If it stalls or skips going up:** a 12 V motor on an ~11 V supply loses torque as speed rises. Lower `LIFT_MAX_SPEED_MM_S` in `lift.h` (try 12) and upload. In Part 9.3, set `track_speed` and `search_speed` in base.yaml to the same value or lower.
3. **Force:** with a **flat palm** on top of the moving head (never fingers in a gap), it should stall rather than push hard.
   - If it pushes too hard, lower Vref (6.5) below 0.18 V in small steps, until it only just lifts reliably.
   - Then raise it about 20 %.
   - **Write the final Vref and speed in your report.**
4. Type `l ?` after a few parks. It shows `last park correction`. If parking regularly corrects more than 1–2 mm, the motor is skipping steps: lower the speed or `LIFT_ACCEL_MM_S2`, or look for tight spots in the mechanism.
5. `quit`.

✅ **Stage C done** when the lift goes the right way, the travel is measured, and speed and force are tuned. The face-tracking behaviour is tested in Part 9.

---

## 9. Stage D — ROS 2: the full behaviour

Everything here is **on the Pi** unless it says otherwise.

### 9.1 Plug in the ESP32

Plug the FireBeetle's USB cable into the **Pi's USB port 2**; the Pico stays on USB port 1. The robot doesn't need WiFi for this.

### 9.2 Give the ESP32 a fixed name: /dev/headcam

Open the udev rules file from Phase 5 Step 8:

```bash
sudo nano /etc/udev/rules.d/99-robot.rules
```

Add this line below the Pico and LiDAR lines:

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", SYMLINK+="headcam"
```

`303a` is Espressif's ID, so it can't clash with the Pico's `2e8a`. Save with `Ctrl+O` then Enter, exit with `Ctrl+X`. Then:

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/headcam                    # should point to ttyACM1 (or similar)
python3 -m serial.tools.miniterm /dev/headcam 115200
```

In miniterm type `faces on` and press Enter: `f ...` lines should stream. Type `faces off`, then exit with `Ctrl+]`.

### 9.3 Update your ROS package

The changes go into your Phase 6 package `$REPO/hospital_robot_base`. If your files still match the Phase 6 guide's Appendix B, the patch does everything:

```bash
cd $REPO
patch -p1 --dry-run < ~/HeadColumnProject_rev2/ros/phase8_ros.diff
```

**You should see** six `checking file ...` lines and no `FAILED`. Then apply it for real and add the new node:

```bash
patch -p1 < ~/HeadColumnProject_rev2/ros/phase8_ros.diff
cp ~/HeadColumnProject_rev2/ros/hospital_robot_base/hospital_robot_base/head_column_node.py \
   $REPO/hospital_robot_base/hospital_robot_base/
```

> **If the dry run says FAILED**, you've edited those files since Appendix B. Compare yours with the complete files, then copy them over yours:
> ```bash
> diff $REPO/hospital_robot_base/hospital_robot_base/pico_bridge.py \
>      ~/HeadColumnProject_rev2/ros/hospital_robot_base/hospital_robot_base/pico_bridge.py
> cp -r ~/HeadColumnProject_rev2/ros/hospital_robot_base/* $REPO/hospital_robot_base/
> ```
> Then re-add any of your own edits you saw in the `diff`.

What changed, in plain words:

| File | Change |
|---|---|
| `pico_protocol.py` | Understands the Pico's `l` lines and the ESP32's `f` lines |
| `pico_bridge.py` | New topic `/lift/command` → sends `l ...` to the Pico. Publishes `/lift/state` and the lift height as `column_joint` on `/column_states`. Sends `l s` when stopped |
| `head_column_node.py` | **New.** Search / track / park behaviour |
| `config/base.yaml` | `column_joint` name, `column_states` added to `joint_state_publisher`, new `head_column:` settings |
| `launch/base.launch.py` | New argument `use_head:=true` starts `head_column` |
| `setup.py` | Registers the `head_column` program |
| `test/test_pico_protocol.py` | 4 new tests |

Now open `$REPO/hospital_robot_base/config/base.yaml` and, in the `head_column:` section, set `soft_max_mm` to the `LIFT_SOFT_MAX_MM` you measured in 8.3. If you lowered the speed in 8.4, also lower `track_speed` and `search_speed`.

Build and test:

```bash
cd ~/robot_ws
colcon build --packages-select hospital_robot_base --parallel-workers 2
source install/setup.bash
colcon test --packages-select hospital_robot_base && colcon test-result --verbose
```

**You should see** 16 tests, 0 failures. **Commit to git.**

### 9.4 The column in your URDF

Your URDF already has `column_joint` (held at 0 until now). The bridge now publishes the real height once the lift is homed. `joint_state_publisher` merges it in, because `base.yaml` now says `source_list: [wheel_states, column_states]`. There's nothing else to configure, and nothing flickers.

Check once in your xacro that:
- `column_joint` is `type="prismatic"` with `axis xyz="0 0 1"`;
- its `<limit upper="...">` equals your travel in metres (e.g. 0.40);
- the camera sits on a link *above* `column_link`, so it moves in RViz.

### 9.5 Test the lift through ROS

Robot on blocks, E-stop within reach. Terminal 1:

```bash
ros2 launch hospital_robot_base base.launch.py use_lidar:=false
```

Terminal 2:

```bash
ros2 service call /pico_bridge/arm std_srvs/srv/SetBool "{data: true}"
ros2 topic pub --once /lift/command std_msgs/msg/String "{data: home}"
ros2 topic echo /lift/state                    # 'l ...' lines, 10 per second (Ctrl+C to stop)
ros2 topic pub --once /lift/command std_msgs/msg/String "{data: goto 150}"
ros2 topic pub --once /lift/command std_msgs/msg/String "{data: park}"
```

**You should see:** the lift homes, goes to 150 mm and parks. Terminal 1 logs `Lift: IDLE homed=1 ...` on each change.

Allowed lift commands: `home`, `park`, `stop`, `info`, `goto <mm> [speed]`.

### 9.6 The full behaviour

Stop the launch (Ctrl+C) and start it again with the head node:

```bash
ros2 launch hospital_robot_base base.launch.py use_head:=true use_lidar:=false
```

Terminal 2:

```bash
ros2 topic echo /head_column/state
```

Terminal 3:

```bash
ros2 service call /pico_bridge/arm std_srvs/srv/SetBool "{data: true}"
```

**You should see** the state go **WAIT_ARM → HOMING → SEARCHING**, and **TRACKING** once you stand in front of the camera. Then try:

```bash
ros2 topic pub --once /head_column/command std_msgs/msg/String "{data: hold}"
ros2 topic pub --once /head_column/command std_msgs/msg/String "{data: search}"
ros2 service call /head_column/park std_srvs/srv/Trigger     # answers once parked
ros2 topic echo /head_column/parked                          # true
```

- Press the E-stop: the state becomes **WAIT_ARM** immediately. After re-arming, the lift homes again.
- In RViz (on the PC), add **RobotModel**: the column moves with the real lift.

**How tracking behaves:** it moves, stops, looks at a *fresh* picture, then corrects ("move and look"). Expect small steps with short pauses, not smooth continuous motion. That's deliberate: it can't overshoot, whatever the camera delay.

Head commands: `search`, `park`, `stop`, `hold`, `home`.

States you'll see:

| State | Meaning |
|---|---|
| WAIT_ARM | Robot not armed: the lift has no power. Nothing moves |
| HOMING | Finding the switch |
| SEARCHING | Sweeping up and down looking for a face (up to 60 s) |
| TRACKING | Following a face |
| WAITING | Holding still, still watching for a face |
| PARKING | Going down onto the switch |
| PARKED | Resting on the switch: **safe to navigate** |
| STOPPED | After `stop` |
| FAULT | Lift fault (message says what). Send `home` to retry |

### 9.7 Tuning

| What you see | Change | Where |
|---|---|---|
| Hunts up and down | Lower `track_gain` (0.5), raise `deadband_mm` (25) | base.yaml `head_column:` |
| Too slow to follow | Raise `track_gain` towards 1.0 | base.yaml |
| Face sits too high/low in the picture | `TARGET_Y_FRACTION` (0.4 = face higher) | ESP32 `config.h` |
| Reacts to people far away | Raise `MIN_FACE_WIDTH_PX` | ESP32 `config.h` |
| Searches too long / too short | `search_timeout` | base.yaml |
| Re-searches too soon after losing a face | `face_lost_timeout` | base.yaml |
| Don't lift by itself after arming | `search_after_home: false` (or `auto_home_on_arm: false`) | base.yaml |
| Stalls when lifting | `LIFT_MAX_SPEED_MM_S` + `track_speed` / `search_speed` | lift.h + base.yaml |

After changing base.yaml, rebuild (the `colcon build` line in 9.3). After changing lift.h, upload the Pico again. After changing the ESP32's config.h, upload the ESP32 again.

### 9.8 Hook it into the touchscreen → Nav2 flow

When the patient confirms a destination, your touchscreen code must:

1. **Call `/head_column/park`.** It only answers when the lift is down on the switch (or with an error). **Never send a Nav2 goal if it fails.**
2. Send the destination to Nav2 and wait until the robot arrives.
3. Publish `search` on `/head_column/command`, so the head rises to find the patient's face again.

`park_then_navigate_example.py` does exactly this with `nav2_simple_commander`:
1. Edit the coordinates in its `DESTINATIONS` list.
2. With Nav2 running and the robot armed, try it:
   ```bash
   python3 ~/HeadColumnProject_rev2/pi/park_then_navigate_example.py pharmacy
   ```
3. Copy its `go_to_destination()` function into your touchscreen program.

### 9.9 Start it automatically

Add `use_head:=true` to however you start `base.launch.py`. Or, in `launch/base.launch.py`, change the default of `use_head` from `'false'` to `'true'`, and rebuild.

On the finished robot, set `#define ENABLE_WIFI 0` in the ESP32's `config.h` and upload. It then boots 15 s faster and makes no WiFi network.

✅ **Stage D done** — the whole Phase 8 chain works.

---

## 10. Troubleshooting

### Uploading and the Arduino IDE

| Problem | Fix |
|---|---|
| No port / `/dev/ttyACM0` missing | Charge-only cable → try another; try another USB port |
| "Permission denied" on the port | Part 2.2 (`dialout` group), then log out/in |
| ESP32: "Failed to connect" | Hold **BOOT**, tap **RST**, release **BOOT**, upload, then press **RST** |
| Uploaded to the wrong board | Check **Tools → Board** and **Port** every time you switch between ESP32 and Pico |
| Error: *Wrong ESP32 board package* | Install version **2.0.17** (Part 2.3) |
| Error: *PSRAM is switched off* | Tools → PSRAM → **OPI PSRAM** |
| "Sketch too big" | Tools → Partition Scheme → **Huge APP** |
| Serial Monitor empty (ESP32) | Tools → USB CDC On Boot → **Enabled**; 115200 baud; press **RST** |
| Pico: errors about `liftCommand` / `line` / `armState` | Names differ in your firmware: see the note at the end of 7.2 |

### Camera and WiFi

| Problem | Fix |
|---|---|
| `[camera] init FAILED, error 0x105` or `0x20001` | Camera ribbon not fully in or the wrong way; **unplug USB for 10 s** and retry |
| Board resets when WiFi starts ("brownout") | Weak USB port or cable: use a better cable or the Pi's USB |
| WiFi won't connect | Must be **2.4 GHz** (not a `_5G` network); name and password are case-sensitive; fallback network `HeadColumn-Setup` → http://192.168.4.1 |
| `cam_hal: EV-VSYNC-OVF` messages | Harmless dropped frame (often while WiFi is struggling). Only if constant *and* the video stutters: in `vision.h` change `20000000` to `10000000` |
| No `f` lines | Type `faces on`; check you uploaded the rev 2 firmware (Part 5.5) |
| No face detected | Better light (not behind the person), face the camera, 0.5–1.5 m away, lower `MIN_FACE_WIDTH_PX`, camera not upside down |

### Lift (Pico)

| Problem | Fix |
|---|---|
| `# lift: not armed` | Arm first (`a 1`, or the arm service): the lift is powered through the relay |
| `unknown command 'l'` | The Pico still runs 1.0.1: add lift.h and upload 1.1.0 (Part 7.2). `fake_pico` also doesn't know `l` |
| `lift_console.py`: cannot open /dev/pico | Stop pico_bridge / miniterm first |
| Armed, but the motor does nothing | RESET+SLEEP tied to 3V3? EN wire on GP12? VMOT present at driver pin 16? Vref not 0 V? |
| Buzzes but doesn't turn | Coil pairs wrong: find the ~30 Ω pairs (6.2) |
| Moves the wrong way | `LIFT_DIR_HIGH_MEANS_UP` in lift.h |
| Moves the wrong distance | `LIFT_MM_PER_REV` / `LIFT_MICROSTEPS` in lift.h |
| `l ?` always says PRESSED | Wire on NO instead of NC, or a broken wire (the fail-safe working) |
| `FAULT 1: switch never triggered` | Switch wiring, or the lift moved UP while homing |
| `FAULT 2: switch still PRESSED` | Stuck lever or broken wire |
| Stalls or skips going up | Lower `LIFT_MAX_SPEED_MM_S` (8.4) |
| "lift position forgotten" after E-stop | By design: any motor power loss means it must re-home |
| Park corrections over 2 mm | Steps being lost: slower speed, lower accel, check the mechanism |
| Driver very hot | Vref too high: set it again (6.5) |

### ROS

| Problem | Fix |
|---|---|
| head_column: "No lift status from the Pico" | pico_bridge not running, or the Pico firmware has no lift.h |
| head_column: "Cannot open /dev/headcam" | udev rule (9.2); is the ESP32 plugged into the Pi? |
| State stays WAIT_ARM | The robot isn't armed: call `/pico_bridge/arm` |
| `/head_column/park` fails "not ARMED" | Same: arm first |
| `patch` says FAILED | See the note in 9.3 |
| Column doesn't move in RViz | Only published once homed; check `source_list` in base.yaml and the joint name `column_joint` |
| Tracking hunts | Lower `track_gain`, raise `deadband_mm` (9.7) |
| Tracking goes the wrong way | Camera upside down → `CAMERA_VFLIP 1` in the ESP32 config.h |
| Follows the wrong person | It follows the biggest (closest) face; raise `MIN_FACE_WIDTH_PX` |

---

## 11. How it works (use this for your report and viva)

### 11.1 Three processors, three jobs

- **ESP32-S3 (column):** camera and face detection. Core 0 runs the detector, core 1 runs the rest. Each frame it sends one `f` line over USB.
- **Raspberry Pi (body):** the `head_column` ROS node decides what to do (search, track, park) and publishes lift commands. `pico_bridge` forwards them to the Pico, because it owns the serial port.
- **Pico W (body):** `lift.h` generates the step pulses from a 20 kHz hardware timer, reads the home switch, and runs homing and parking.

Putting the motor on the Pico keeps the only moving cable to a single USB lead. It also puts the real-time and safety-critical parts on the board that already owns the E-stop and relay.

### 11.2 Face detection

Espressif's **esp-dl** library (bundled in ESP32 Arduino core 2.0.x) provides a two-stage convolutional neural network detector. `HumanFaceDetectMSR01` proposes candidate boxes on a scaled-down image, and `HumanFaceDetectMNP01` refines them. It works directly on the camera's raw RGB565 pixels.

The firmware keeps the **largest** box (the closest person) and ignores faces narrower than `MIN_FACE_WIDTH_PX`. The Pi only believes a face after `confirm_frames` frames in a row, which rejects one-frame false detections.

### 11.3 Turning pixels into millimetres (distance-independent)

The camera doesn't know how far away the person is, but it can see how wide their face is. With a simple pinhole camera model (focal length *f*, distance *d*):

```
vertical offset / d  = error_px / f
face width      / d  = face_px  / f
⇒ vertical offset (mm) = error_px × (face width mm ÷ face_px)
```

The distance cancels out. With an average face about 150 mm wide (`AVERAGE_FACE_WIDTH_MM`), one measurement gives the height correction in millimetres directly, whether the patient is 0.5 m or 1.5 m away.

### 11.4 Dealing with delay: "move and look"

A face measurement is old by the time it reaches the Pi: inference time plus USB. Correcting continuously from old pictures would make the lift overshoot.

So the Pi uses **move and look**:
1. Command a correction: `target = height + track_gain × error`.
2. Wait until the Pico reports the lift has stopped.
3. Note the camera frame number, and wait for a frame numbered at least `settle_frames` (2) higher. That picture was certainly taken after the lift stopped.
4. Use that frame to decide the next correction, or do nothing if the error is inside the deadband.

In simulation with a 150 ms camera delay, it settled on a new face height in 2 corrections. It sent just 1 extra command in 20 s of steady tracking, so there was no hunting.

### 11.5 Motor control (`lift.h` on the Pico)

- **Step generation:** a phase accumulator (DDA). `add_repeating_timer_us` runs an interrupt every 50 µs. It adds the current speed to an accumulator, and each overflow sends one STEP pulse.
- **Smooth motion:** a trapezoidal profile. The interrupt compares the distance to go with the stopping distance *v² / 2a* and accelerates or brakes accordingly. Targets can change mid-move, and it smoothly re-plans, including reversing.
- **Integer maths only:** the RP2040 has no floating-point unit, so the interrupt uses 16.16 fixed-point numbers.
- **Tested in simulation:**
  - homing and goto work, and the soft limits hold;
  - stop works mid-move;
  - parking with 4 mm of injected lost steps corrected exactly 4.0 mm;
  - power loss mid-move is handled;
  - a broken switch wire and a missing switch both fault safely.

### 11.6 Safety layers

| Layer | How |
|---|---|
| Arming | Never automatic: a human arms the robot |
| Power | The lift's VMOT comes through the E-stop and relay: **no arming, no lift power** |
| E-stop | Cuts VMOT in hardware, and calls `liftEmergencyStop()` in firmware |
| After power loss | Position forgotten → must home again |
| Mechanical force | Small 2.6 kg·cm motor, current set by Vref, slow speed |
| Home switch | Checked **every 50 µs** in the interrupt: downward steps refused while pressed |
| Switch noise | 2 ms debounce |
| Broken wire | NC wiring reads as "pressed", so it won't drive down |
| Top | Software limit at `LIFT_TRAVEL_MM` once homed (a top switch would add a hardware limit) |
| Parking | Every park re-homes and reports the correction |
| Faults | Switch never found / stuck → stop, FAULT, needs `l h` |
| Navigation | ROS only drives after `/head_column/park` confirms "parked" |

### 11.7 State machines

- **Pi (`head_column`):** WAIT_ARM → HOMING → SEARCHING ⇄ TRACKING / WAITING → PARKING → PARKED, plus STOPPED and FAULT (table in 9.6).
- **Pico (`lift.h`):** IDLE, MOVING, HOMING, PARKING, FAULT. The Pico only moves when told to, and always within its own limits.

### 11.8 Protocol reference

| Line / topic | Direction | Meaning |
|---|---|---|
| `f <frame> <found> <cx> <cy> <w> <err_mm> <fps>` | ESP32 → Pi (USB) | one per camera frame, after `faces on` |
| `l h` / `l p` / `l g <mm> [v]` / `l s` / `l ?` | Pi → Pico | home / park / go to / stop / info |
| `l <state> <homed> <pos> <target> <switch> <fault>` | Pico → Pi | lift status, 10 per second |
| `/lift/command`, `/lift/state` (String) | ROS | through pico_bridge |
| `/column_states` (JointState) | ROS | lift height → `/joint_states` |
| `/head_column/command`, `/head_column/park`, `/head_column/state`, `/head_column/parked` | ROS | used by the touchscreen |

The ESP32's web pages (ports 80/81) still work for the browser preview, but the robot doesn't use them.

### 11.9 Things worth measuring for your report

- Detection frame rate (`fps`): one-stage vs two-stage (`FACE_DETECT_TWO_STAGE`).
- Maximum detection distance (face width in pixels vs distance).
- Homing repeatability: park 20 times and record `last park correction` (`l ?`).
- Tracking settle time: from a face appearing to the lift stopping.
- Final Vref, speed, and whether the lift stalls against a palm.

### 11.10 Known limitations / future work

- **Open-loop stepper:** it can't feel a stall. Every park re-homes and reports the correction. The DRV8825's FAULT pin (not wired) or an encoder on the lift would add feedback.
- **Top switch:** a second switch at the top would add a hardware limit. Every Pico GPIO is already used on page 7, so it would need a pin freed up (or an I/O expander).
- **Hardware interlock:** the switch is checked in software (very fast, but software). A logic circuit that blocks only *downward* steps when the switch is pressed would add a hardware layer. Wiring the switch straight into EN would stop the lift ever leaving the switch.
- **Detection quality:** works best with frontal, well-lit faces. If that isn't good enough, the plan's **Option B** (stream to the Pi and detect with OpenCV/MediaPipe) could reuse the same lift commands and state machine.

---

## Final safety checklist before running near people

- [ ] DRV8825 Vref set (0.20 V or lower) with the motor unplugged; motor reconnected only with the power off
- [ ] RESET + SLEEP tied to 3V3; R16 pull-up on EN fitted; C3 470 µF at the driver
- [ ] Home switch on **NC**; `l ?` shows PRESSED when pressed *and* when the wire is unplugged
- [ ] `l h` goes DOWN; `LIFT_TRAVEL_MM`, `LIFT_SOFT_MAX_MM` and `soft_max_mm` measured on the real column
- [ ] Speed and force tested: it stalls against a flat palm (8.4)
- [ ] E-stop stops the lift and the wheels; the lift re-homes after re-arming
- [ ] ESP32 USB cable has a slack loop and can't catch in the lead screw
- [ ] Touchscreen code only starts Nav2 after `/head_column/park` succeeds
- [ ] Wiring diagram page 3 homing note corrected (HIGH = at home)
