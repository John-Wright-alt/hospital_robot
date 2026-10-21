# Phase 7 — SLAM, Nav2, ultrasonics and the full guiding demo

**For:** the assembled robot (Pi 4 + Pico W + LD06 + head column), Ubuntu 24.04, ROS 2 Jazzy.
**Written:** 18 Oct 2026, while the 12 V battery was flat. **Updated the same day** after you uploaded the Pi's current `pico_bridge.py`, `pico_protocol.py`, `head_column_node.py`, `base.launch.py`, `base.yaml` and `test_speech_logic.py`, and told me about the wheel-sign change (Pico) and `ENABLE_WIFI 0` (ESP32). Nothing in this package has run on the robot yet.

This guide takes you from "battery charged" to the video: Anna wakes up, asks where the person wants to go, guides them there, offers to wait, and comes back with them or alone.

Work through the stages in order. Each stage ends with a **pass condition**. Don't start the next stage until the current one passes.

| Stage | What | Battery needed? |
|---|---|---|
| 0 | Install the files, apply the patches, build, run the tests | No |
| 1 | Desk rehearsal of the whole touchscreen flow (simulated mission) | No |
| 2 | Measure the ultrasonics, meter-check the ECHO dividers, type in the positions | No (power OFF for the meter check) |
| 3 | Short power-up checks: battery, odometry fix, LiDAR direction | Yes (≈20 s of motor power) |
| 4 | Ultrasonics: flash firmware 1.2.0 from the PC, test each sensor | Yes |
| 5 | Mapping your apartment (roadmap 7.1) | Yes |
| 6 | Waypoints: home, doctor, X-ray, pharmacy | Yes |
| 7 | Navigation tests: blocks, then floor (roadmap 7.2, 7.3) | Yes |
| 8 | Mission tests with `go_to` | Yes |
| 9 | Full demo and video | Yes |

---

## Status before you start

What I could check here (in a cloud container **without ROS 2**) and what I could not:

| Item | Status | Evidence / what is missing |
|---|---|---|
| Firmware 1.2.0 (`ultrasonic.h` + patch) | **Implemented, compiles, not tested on hardware** | Compiled for `rp2040:rp2040:rpipicow` (core 6.3.0, Simple FOC 2.4.0) against the project copy of your firmware. Never run on a Pico. |
| Bridge patch (ultrasonic + `/odom` speed fix) | **Implemented, tested with simulated serial lines only** | Applied cleanly to **the Pi's own `pico_bridge.py`** (uploaded 18 Oct, with the Phase 8 lift code). With a recorded-style noisy input: still-standing speed peak 0.0686 → 0.0015 m/s; moving speed unchanged (0.1440 m/s). Not run under real ROS. |
| URDF patch (3 sensor frames) | **Implemented, expands with xacro. Positions are PLACEHOLDERS** | Mounting positions/directions are not in any project document. You measure them in Stage 2. |
| `hospital_robot_nav` package | **Implemented, 33 unit tests pass, never run under ROS 2** | Mission logic, waypoints, scan filter maths, config consistency tested. Nodes and launch files only syntax/stub-checked. |
| Nav2 parameters (`nav2_params_real.yaml`) | **Written, not tested** | Built from your sim file (the installed Jazzy default). RPP controller, ultrasonic range layer and collision monitor are new. Jazzy may warn about or reject a parameter I got wrong — Stage 7 catches that. |
| HRI trip flow (`nav_flow.py` + patch) | **Implemented, 98 HRI tests pass (19 new), screens rendered in a browser** | Includes your Pi copy of `test_speech_logic.py`, run against the patched `phrases.yaml` (so `hri_speech` accepts the new sentences). Not run on the Pi. |
| Ultrasonic sensors | **Not tested** | ECHO dividers recorded as "fitted" in `Robot_Wiring.kicad_sch` (1k/2k → 3.33 V). Which physical sensor is US1/US2/US3 is unconfirmed. |
| `/odom` false-speed issue | **Fix implemented, not verified on the robot** | Stage 3 verifies it with the measurement from `KNOWN_ISSUE_ODOM_FALSE_SPEED.md`. |
| Wheel-sign change in the Pico firmware (18 Oct) | **Partially verified** | You report the robot now drives forwards. The turning direction and which wheel is reported as "left" have not been checked since the change. Stage 3.4 does that **before mapping**. |
| ESP32 `ENABLE_WIFI 0` | **No effect on Phase 7** | Face data goes over USB (`/dev/headcam`); nothing in this package uses the ESP32's WiFi. |
| Mapping, localisation, navigation | **Not started — blocked by the battery** | |

**About your files:** the bridge patch is now checked against the Pi's own `pico_bridge.py`, and the HRI patch against `hospital_robot_hri` (which `PHASE9_HRI_STATUS.md` says matches the Pi). The firmware and URDF patches are still checked only against the 9 Oct project copies. Your firmware now has different wheel signs; the patch doesn't touch those lines, so that shouldn't matter. That's why every patch looks for short anchor lines, shows a dry run first, and **writes nothing** if an anchor is missing. If one refuses, send me the file and I'll adapt the patch.

---

## What's in the package

```
phase7/
├── PHASE7_GUIDE.md                       this file
├── patches/                              run these; they edit YOUR files (with backups)
│   ├── patchlib.py
│   ├── apply_firmware_patch.py           pico_motor_firmware 1.1.0 -> 1.2.0 (ultrasonic)
│   ├── apply_bridge_patch.py             pico_bridge: /ultrasonic/* + /odom speed fix
│   ├── apply_urdf_patch.py               robot.urdf.xacro: include ultrasonic.xacro
│   └── apply_hri_patch.py                Anna: connect to the mission node
├── firmware/pico_motor_firmware/ultrasonic.h
├── ros/
│   ├── hospital_robot_base/              ultrasonic.py + test (copied by the bridge patch)
│   ├── hospital_robot_description/urdf/ultrasonic.xacro
│   └── hospital_robot_nav/               NEW package (copy it into ~/robot_ws/src)
│       ├── config/nav2_params_real.yaml  Nav2 for the real robot
│       ├── config/slam_mapping.yaml      SLAM Toolbox
│       ├── config/mission.yaml           guide/alone speeds, safety rules
│       ├── config/waypoints_template.yaml
│       ├── behavior_trees/navigate_no_backup.xml   recoveries without reversing/spinning
│       ├── rviz/phase7_sensors.rviz      RViz view: robot, LiDAR, the 3 ultrasonic cones
│       ├── launch/mapping.launch.py
│       ├── launch/navigation.launch.py
│       └── hospital_robot_nav/           scan_body_filter, mission, go_to, save_waypoint,
│                                         ultrasonic_check, simulate_mission (+ pure logic)
└── hri/hospital_robot_hri/               nav_flow.py + test (copied by the HRI patch)
```

### How the pieces talk

```
 touchscreen ─tap/choice─► hri_interaction (+ nav_flow) ──/mission/command──► mission ──NavigateToPose──► Nav2
                                     ▲                                          │   ▲
                                     └──────────────/mission/status─────────────┘   │ /robot_state (ARMED?)
                                                                                    │ /head_column/state (PARKED?)
 LD06 /scan ─► scan_body_filter ─► /scan_filtered ─► AMCL, costmaps, collision monitor
 Pico 'u' lines ─► pico_bridge ─► /ultrasonic/{left,front,right} ─► range layer, collision monitor
 Nav2 controller ─► velocity_smoother ─► collision_monitor ─► /cmd_vel (TwistStamped) ─► pico_bridge ─► Pico
```

### Safety rules built into the software (on top of the E-stop and the Pico's own limits)

* **Nothing here ever arms the robot.** A person arms it with the `/pico_bridge/arm` service, as before.
* The mission node **refuses to send a goal** unless `/robot_state` is `ARMED` (and fresh, < 3 s old) **and** the head column reports `PARKED`. It **cancels** the goal the moment either stops being true.
* Speed: 0.25 m/s guiding, 0.35 m/s alone. The velocity smoother caps everything at 0.35 m/s and 0.8 rad/s. The Pico still clamps at 9 rad/s (0.65 m/s).
* The robot **never reverses or spins** as a recovery (custom behaviour tree): a person walks behind it and there is no rear sensor.
* Collision monitor: anything within ~12 cm in front of the front edge → stop; within ~40 cm → 40 % speed. It reads the raw LiDAR and ultrasonics, independent of the planner.
* If the ultrasonic data stops for 2 s, the local costmap goes "not current" and Nav2 stops the goal. A dead sensor fails safe (it stops the robot — the downside is it also stops the demo; see Troubleshooting).
* The on-screen **Stop** button is a convenience. **The E-stop is the safety stop.** Keep a helper on it for every test that moves the robot.
* The robot only drives without a fresh tap in two cases, both returning home alone: nobody answers at the destination within 45 s, or the person never comes back within 10 min. Set `arrive_timeout_s` / `wait_timeout_s` to `0.0` to switch that off.

---

## Stage 0 — Install, patch, build, test (no battery needed)

Do this on **the Pi** (the robot runs everything) and **the Dell** (for RViz and the desk rehearsal). The Pi can run on the wall adapter for this stage (as in `phase9.txt`).

### 0.1 Safety net first: commit

```bash
cd ~/robot_ws/src
git status
git add -A && git commit -m "Before Phase 7"
```

**Why:** every patch also makes `.bak_phase7_<time>` copies, but a commit is the clean way back.
If `~/robot_ws/src` is not a git repository, copy it instead: `cp -a ~/robot_ws/src ~/robot_ws_src_before_phase7`.

### 0.2 Unzip

```bash
cd ~
unzip phase7_package.zip -d ~/phase7
ls ~/phase7
```

**Expected:** `PHASE7_GUIDE.md  firmware  hri  patches  ros`.

### 0.3 Install the ROS packages Nav2 needs (Pi; the Dell has them from simulation)

```bash
ros2 pkg list | grep -E "^(nav2_bringup|slam_toolbox|nav2_collision_monitor|nav2_regulated_pure_pursuit_controller|teleop_twist_keyboard)$"
```

**Expected:** all five names. If any are missing:

```bash
sudo apt update
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox ros-jazzy-teleop-twist-keyboard
```

### 0.4 New package `hospital_robot_nav`

```bash
cp -r ~/phase7/ros/hospital_robot_nav ~/robot_ws/src/
mkdir -p ~/robot_ws/maps
```

### 0.5 Patches — dry run first, read the diff, then apply

Each script shows exactly what it will change and writes nothing until you add `--apply`.

**Bridge** (ultrasonic topics + `/odom` speed fix):

```bash
cd ~/phase7/patches
python3 apply_bridge_patch.py
```

Read the diff. You should see 5 changes listed (`+ import UltrasonicPublisher` … `+ /odom twist from ticks…`).

* If it says `anchor not found` for the **odom** lines: you already changed that code (perhaps you fixed the known issue yourself). Then use `python3 apply_bridge_patch.py --skip-odom-fix` and tell me how you fixed it.

```bash
python3 apply_bridge_patch.py --apply
```

**URDF** (three sensor frames):

```bash
python3 apply_urdf_patch.py
python3 apply_urdf_patch.py --apply
```

**HRI** (Anna ↔ mission node):

```bash
python3 apply_hri_patch.py
python3 apply_hri_patch.py --apply
```

**Firmware:** only the files on the Dell (you flash from the Dell). Use **your current sketch, with the new wheel signs** — not the old project copy. The patch doesn't touch `LEFT_FORWARD_SIGN` / `RIGHT_FORWARD_SIGN`. Run it there, pointing at your sketch folder:

```bash
python3 apply_firmware_patch.py --sketch ~/robot_ws/src/firmware/pico_motor_firmware
```

Don't apply it yet — that's Stage 4, with a backup of the working 1.1.0 firmware.

**If any patch says `PATCH NOT APPLIED`:** nothing was written. Send me the file it names.

### 0.6 Build

```bash
cd ~/robot_ws
colcon build --symlink-install --packages-select hospital_robot_nav hospital_robot_base hospital_robot_description hospital_robot_hri
source install/setup.bash
```

**Expected:** `Summary: 4 packages finished`. Warnings about `setup.py install is deprecated` are normal.

### 0.7 Unit tests (nothing moves)

```bash
cd ~/robot_ws/src/hospital_robot_nav && python3 -m pytest test -q
cd ~/robot_ws/src/hospital_robot_hri && python3 -m pytest test -q
cd ~/robot_ws/src/hospital_robot_base && python3 -m pytest test -q
```

**Expected:** `33 passed` for the nav package. For the HRI package: all pass, including `test_nav_flow.py` (19 tests). In this container, 98 HRI tests passed (`test_interaction_logic.py`, `test_face_logic.py`, `test_nav_flow.py` and your Pi copy of `test_speech_logic.py`). I couldn't run `test_audio_player.py` here.

### 0.8 Re-render Anna's audio (Pi)

The HRI patch added five sentences and changed the `arrived` sentence:

```bash
ros2 run hospital_robot_hri audio_tool render
```

**Pass condition, Stage 0:** 4 packages build, all tests pass, nothing refused.

---

## Stage 1 — Desk rehearsal of the whole flow (no battery, nothing moves)

**Objective:** see and hear the complete guiding conversation before the robot can drive, using a **pretend** mission node and the **simulated** head column. Run it **on the Pi** (on the wall adapter is fine). Use the Dell's web browser as the touchscreen.

**Terminal 1** (a Pi terminal: `ssh wright@robot.local`, then `source /opt/ros/jazzy/setup.bash && source ~/robot_ws/install/setup.bash`):

```bash
ros2 launch hospital_robot_hri hri.launch.py sim_head:=true head_scenario:=normal nav:=true remote_touch:=true
```

**Terminal 2** (another Pi terminal, same two start lines):

```bash
ros2 run hospital_robot_nav simulate_mission --trip-s 10
```

It prints `SIMULATED mission node … Nothing moves.`

**Browser on the Dell:** open `http://robot.local:8080/?dev=1`. Click the eyes to tap (`remote_touch:=true` lets clicks from the Dell count). There's no sound without the battery: the speaker amplifier is on Rail B. The captions still show what Anna says.

**Walk through this, and tick each line:**

1. Tap → "Hello, I am Anna…" → simulated adjusting → menu "Where would you like to go?"
2. Press **Pharmacy** → "The pharmacy. Thank you. I am lowering my screen…"
3. The trip bar appears: **"Taking you to the Pharmacy"** with a red **Stop**. Anna says "Please follow me…".
4. After ~10 s: "We have arrived…" and the menu **Wait for me here / Go back without me**.
5. **Wait for me here** → "I will wait for you right here…" → menu **Take me back to reception / Go back without me**.
6. **Take me back to reception** → trip bar "Taking you back to reception" → after ~10 s "We are back at reception. Thank you, and take care." → sleeping eyes.
7. Repeat, but press **Stop** during the trip → "I have stopped…" → menu **Continue / Take me back / Go back without me** + "Stay here".
8. Repeat with **Go back without me** at the destination → trip bar "Going back to reception" → sleeps on arrival, without speaking.
9. Don't touch anything at the destination for 45 s → it goes back alone by itself.
10. Stop terminal 2, start `ros2 run hospital_robot_nav simulate_mission --scenario fail` → every trip ends with "I can't reach the destination…" and **Try again / Go back without me** + "Stay here".
11. `--scenario notarmed` → "My motors are switched off, so I cannot drive…".

Watch the state too: `ros2 topic echo /hri/interaction/state` (shows `trip` and `remaining_m`).

**Pass condition, Stage 1:** all 11 behave as described. If a sentence is missing, check the `hri_speech` log for `unknown event` (audio not re-rendered, or phrases patch not applied).

> Note: the destination menu is shown with the column **parked** (the screen low). The robot must not drive with the screen raised (Phase 8 rule), so after arriving the screen stays low. Raising it again at the destination is a possible later improvement.

---

## How to use the rest of this guide (read once)

From here on, every command is in its own grey box. Copy the whole box and paste it into the terminal named above it. **Don't type commands that are only mentioned in normal text** — everything you need to run is in a box.

### Terminal names

You work from the **Dell (PC)**. Some terminals run commands on the PC itself. Others are logged in to the **Pi** with `ssh`.

| Name | What it is | How to open it |
|---|---|---|
| **PC-A**, **PC-B** | A terminal on the Dell | `Ctrl+Alt+T`, then run the "PC terminal start" box below |
| **Pi-1**, **Pi-2**, **Pi-3**, **Pi-4** | A terminal on the Dell, logged in to the Pi | `Ctrl+Alt+T`, then run the "Pi terminal start" box below |

**Pi terminal start** — run these lines first, every time you open a Pi terminal:

```bash
ssh wright@robot.local
```

```bash
source /opt/ros/jazzy/setup.bash && source ~/robot_ws/install/setup.bash
```

After `ssh` the prompt changes to `wright@robot:~$`. From then on, everything in that window runs **on the Pi**.

**PC terminal start** — run this first, every time you open a PC terminal:

```bash
source /opt/ros/jazzy/setup.bash
```

**One-time check that the PC and the Pi can see each other.** Run this in a Pi terminal and in a PC terminal. Both must print the **same** number (or both nothing):

```bash
echo $ROS_DOMAIN_ID
```

### Commands you will use again and again

To save repeating the explanation, these are the same everywhere in the guide:

| What | Command | Where |
|---|---|---|
| **Arm** (motor power on; the column homes down) | `ros2 service call /pico_bridge/arm std_srvs/srv/SetBool "{data: true}"` | a Pi terminal |
| **Disarm** (motor power off) | `ros2 service call /pico_bridge/arm std_srvs/srv/SetBool "{data: false}"` | a Pi terminal |
| Clear a latched fault (after `FAULT`) | `ros2 service call /pico_bridge/clear_faults std_srvs/srv/Trigger` | a Pi terminal |
| Stop a running program | press `Ctrl+C` in its terminal | — |

The guide always writes the full command in a box when you need it. The table is only so you recognise them.

### Power switches (from your wiring)

| Switch | What it powers |
|---|---|
| **Green** (main) | Pi (Rail A), sensors, speaker (Rail B), Pico logic through the Pi's USB |
| **Blue** (motor) | Motor and lift power — but only after you **arm**. Off = nothing can move |

**Never** plug in the Pi's USB-C wall adapter while the green switch is on (Phase 6C rule).

---

## Stage 2 — Ultrasonic sensors: measure and check the wiring (robot OFF)

**Objective:** tell the software exactly where each ultrasonic sensor sits on the robot, and make sure the 5 V ECHO signals can't damage the Pico.

**Battery:** not needed. In 2.4 the Pi must be on: use the wall adapter with the green switch **off**, or the battery with the green switch on. **Motors:** never on in this stage.

### 2.1 Understand the reference point

ROS describes every sensor position from one point on the robot, called `base_footprint`. On your robot it is:

> **On the floor, exactly halfway between the two drive wheels**
> (halfway between the centres of the two tyres, under the axle).

The three directions:

| Letter | Direction | Positive (+) means |
|---|---|---|
| **x** | front–back | **in front** of the drive-wheel axle |
| **y** | left–right | to the robot's **left** (standing behind the robot, looking where it drives) |
| **z** | height | **above the floor** |

Top view of the robot:

```
                         FRONT  (+x)
          +-----------------[F]-----------------+
          |                                     |
          |                                     |
         [L]                                   [R]
          |                                     |
          |  [=]-------------- o --------------[=]  <- drive wheels and axle
          |                   (o = base_footprint, on the floor)
          |                                     |
          +-------------------------------------+
                          BACK
     +y points to the LEFT side.   [L] faces left, [F] faces forward, [R] faces right.
```

The **sensor point** you measure to is:

* **Front sensor:** the front face of its two round silver "eyes", at the middle between them.
* **Left and right sensors:** the outer face of their two silver "eyes", at the middle between them.

### 2.2 Measure (tape measure, robot on the floor, wheels pointing straight ahead)

Your wheel separation is **245 mm** (centre to centre, `base.yaml`) and the tyres are **29 mm** wide. So the **outer face of each tyre is 137 mm** from the centre line (122.5 + 14.5). That's why 0.137 appears in the formulas below: you measure from a tyre's outer face, which you can reach easily.

Write your numbers in this table (in **millimetres** first):

| # | Measurement | How to measure | Your value (mm) |
|---|---|---|---|
| A | Front sensor: forward distance | From the **centre of a drive-wheel hub** (the axle), straight forward along the side of the robot, to the line of the front sensor's eye faces. Hold a book flat against the front sensor to make that line easy to measure to. | |
| B | Front sensor: sideways | From the **outer face of the LEFT tyre**, sideways towards the robot's middle, to the middle between the front sensor's two eyes | |
| C | Front sensor: height | Floor to the middle of the eyes | |
| D | Left sensor: forward distance | From the drive-wheel hub, along the side, to the middle between the left sensor's eyes. **Negative** if the sensor is behind the axle | |
| E | Left sensor: sideways | From the **outer face of the LEFT tyre**, straight out sideways, to the face of the left sensor's eyes (book flat against them) | |
| F | Left sensor: height | Floor to the middle of the eyes | |
| G | Right sensor: forward distance | Same as D, on the right side | |
| H | Right sensor: sideways | Same as E, from the outer face of the **RIGHT** tyre | |
| I | Right sensor: height | Floor to the middle of the eyes | |

Now convert to the numbers the software needs (**metres**, mm ÷ 1000):

| Sensor | x | y | z | yaw (fixed) |
|---|---|---|---|---|
| left | D ÷ 1000 | **+**(0.137 + E ÷ 1000) | F ÷ 1000 | `1.5708` (faces left) |
| front | A ÷ 1000 | 0.137 − B ÷ 1000 | C ÷ 1000 | `0.0` (faces forward) |
| right | G ÷ 1000 | **−**(0.137 + H ÷ 1000) | I ÷ 1000 | `-1.5708` (faces right) |

**Worked example** (made-up numbers): E = 80 mm → left y = 0.137 + 0.080 = **0.217**. B = 137 mm (sensor centred) → front y = 0.137 − 0.137 = **0.0**.

**Sanity check** of your results:
* front x should be around **0.17–0.21**
* left y around **+0.20 to +0.24**, and right y around **−0.20 to −0.24** (your side panels are ~75 mm outside the tyres)
* every z is the height you measured, e.g. about **0.10**

If a number is far outside these, measure again.

### 2.3 Meter check of the ECHO dividers (everything OFF)

**Why:** the Pico's pins are **not 5 V tolerant**, and each HC-SR04's ECHO output is a 5 V pulse. Your schematic has a 1 kΩ / 2 kΩ divider on each ECHO line (3.33 V at the Pico) and notes "All three ECHO dividers fitted". Firmware 1.1.0 never triggers the sensors, so today no pulses arrive. Firmware 1.2.0 (Stage 4) starts them. This check proves the resistors are really there before that happens.

**Robot fully off:** battery disconnected, green switch off, Pi not powered, Pico USB unplugged.

Multimeter on **resistance (Ω, 20 kΩ range)**:

| Sensor | Probe 1 | Probe 2 | Expected |
|---|---|---|---|
| US1 (left) | `Echo` pin on the HC-SR04 | Pico **pin 22** (GP17) | ≈ **1 kΩ** |
| US1 (left) | Pico **pin 22** (GP17) | Pico **pin 23** (GND) | **0.6–2.2 kΩ** (not 0) |
| US2 (front) | `Echo` pin on the HC-SR04 | Pico **pin 27** (GP21) | ≈ **1 kΩ** |
| US2 (front) | Pico **pin 27** (GP21) | Pico **pin 28** (GND) | **0.6–2.2 kΩ** |
| US3 (right) | `Echo` pin on the HC-SR04 | Pico **pin 31** (GP26) | ≈ **1 kΩ** |
| US3 (right) | Pico **pin 31** (GP26) | Pico **pin 33** (GND) | **0.6–2.2 kΩ** |

(Pico pins are counted 1–20 down one side and 21–40 back up the other, starting next to the USB socket.)

* **~0 Ω** in a "≈ 1 kΩ" row → the series resistor is missing or bridged. **Stop. Don't do Stage 4.** Tell me.
* **OL / open** in a "≈ 1 kΩ" row → broken wire to that sensor. Fix it before Stage 4.
* The US1/US2/US3 → left/front/right names come from the schematic. If your wires turn out to go to a different sensor, Stage 4.6 sorts that out in software.

### 2.4 Type your measurements into the robot description (Pi)

Power the Pi again: **either** the USB-C wall adapter (green switch **off**), **or** battery + green switch on (blue switch **off**). Never both. Wait ~1 minute for it to boot.

**PC-A** — log in to the Pi:

```bash
ssh wright@robot.local
```

First check that the Stage 0 patch added the sensor file (you should see one line with `ultrasonic.xacro`):

```bash
grep -n "ultrasonic" ~/robot_ws/src/hospital_robot_description/urdf/robot.urdf.xacro
```

Open the sensor file in the `nano` editor:

```bash
nano ~/robot_ws/src/hospital_robot_description/urdf/ultrasonic.xacro
```

Scroll down with the arrow keys to these 12 lines (about line 34). They contain the placeholder numbers:

```xml
  <xacro:property name="us_left_x"    value="0.10"/>
  <xacro:property name="us_left_y"    value="0.19"/>
  <xacro:property name="us_left_z"    value="0.10"/>
  <xacro:property name="us_left_yaw"  value="1.5708"/>

  <xacro:property name="us_front_x"   value="0.18"/>
  <xacro:property name="us_front_y"   value="0.0"/>
  <xacro:property name="us_front_z"   value="0.10"/>
  <xacro:property name="us_front_yaw" value="0.0"/>

  <xacro:property name="us_right_x"   value="0.10"/>
  <xacro:property name="us_right_y"   value="-0.22"/>
  <xacro:property name="us_right_z"   value="0.10"/>
  <xacro:property name="us_right_yaw" value="-1.5708"/>
```

Replace **only the numbers between the quotes** with your values from the 2.2 table (metres, with a dot, e.g. `0.217`, and a minus sign for the right y). Leave the three yaw lines as they are. Your sensors face exactly left, forward and right.

Save and close nano:
1. Press `Ctrl+O`, then `Enter` (saves).
2. Press `Ctrl+X` (exits).

Rebuild the description package (still on the Pi):

```bash
cd ~/robot_ws && colcon build --symlink-install --packages-select hospital_robot_description
```

**Expected:** `Summary: 1 package finished`.

Check that your numbers arrived. This prints the three sensor positions:

```bash
source ~/robot_ws/install/setup.bash && xacro ~/robot_ws/src/hospital_robot_description/urdf/robot.urdf.xacro | grep -A3 'joint name="us_'
```

**Expected:** three blocks like `<origin xyz="0.18 0.0 0.1" rpy="0 0 0.0"/>` with **your** numbers.

Save the change in git (still on the Pi):

```bash
cd ~/robot_ws/src && git add -A && git commit -m "Phase 7: measured ultrasonic positions"
```

**Pass condition, Stage 2:** all six meter readings OK; the `xacro` check shows your measured numbers.

---

## Stage 3 — Power-up checks (short, to save the battery)

**Objective:** confirm the battery, the `/odom` fix and the LiDAR direction before anything drives. The only test with motor power takes about **20 seconds**.

You already verified the turning direction in Phase 6C (`j` = anticlockwise, `l` = clockwise). **If that test was done after you changed the wheel signs (18 Oct)**, step 3.4 only needs the forward check. If it was done **before** the sign change, also do the `j` check in 3.4 (10 more seconds).

**Robot on blocks** (drive wheels off the floor). Helper's hand near the E-stop.

### 3.1 Power on

1. Battery connected, **green** switch **on**, **blue** switch **off**.
2. Wait ~1 minute for the Pi to boot.

### 3.2 Start the robot base

**One-time fix (19 Oct): let the LiDAR driver restart itself.** The LD06 driver sometimes quits right at start-up with `ldlidar communication is abnormal`, followed by `process has died`. Then there is no `/scan` and no dots in RViz. Your `gaze_test.launch.py` already restarts it after 3 s; `base.launch.py` doesn't yet. Do this once, in **Pi-1**:

Back up the launch file:

```bash
cp ~/robot_ws/src/hospital_robot_base/launch/base.launch.py ~/robot_ws/src/hospital_robot_base/launch/base.launch.py.bak_respawn
```

Add `respawn=True, respawn_delay=3.0` to the LiDAR node:

```bash
sed -i "s/name='ldlidar', output='screen', parameters=\[base_params\],/name='ldlidar', output='screen', parameters=[base_params],\n             respawn=True, respawn_delay=3.0,   # LD06 driver can quit at start-up/" ~/robot_ws/src/hospital_robot_base/launch/base.launch.py
```

Check it (expect one line with `respawn=True`):

```bash
grep -n "respawn" ~/robot_ws/src/hospital_robot_base/launch/base.launch.py
```

Rebuild and commit:

```bash
cd ~/robot_ws && colcon build --symlink-install --packages-select hospital_robot_base && source install/setup.bash
```

```bash
cd ~/robot_ws/src && git add -A && git commit -m "base.launch: respawn the LD06 driver"
```

**Every time, before starting the base: wait for the Pi's clock to sync.** The Pi has no clock battery. Its time jumps to the correct value about a minute after boot, and that jump also makes the LD06 driver quit (Phase 9 gaze guide). **Pi-1:**

```bash
until [ "$(timedatectl show -p NTPSynchronized --value)" = yes ]; do echo "waiting for clock sync..."; sleep 2; done; echo "clock OK"
```

**Pi-1** — start the base:

```bash
ros2 launch hospital_robot_base base.launch.py use_head:=true
```

**Expected** in Pi-1 (scrolls past within ~10 s):
* `Connected to Pico on /dev/pico`
* `ticks_per_wheel_rev matches the firmware (15360)`
* `Ultrasonic: publishing ultrasonic/left, ultrasonic/front, ultrasonic/right ... Needs Pico firmware 1.2.0.` (no ultrasonic data yet: that comes with Stage 4)
* `[head] STARTING -> WAIT_ARM`
* **no** `process has died [... ldlidar_stl_ros2_node ...]`. If it appears once, followed ~3 s later by `ldlidar node start is success` again, that's the respawn working: fine. If it keeps repeating, the LiDAR itself has a problem (see 3.5).

**You can ignore** `TICK SCALE CHECK: ticks say the wheels turned 0.01x …` while the robot stands still. That self-check compares the ticks with the Pico's own noisy speed readings: the same noise as the known `/odom` issue. `/odom` no longer uses those readings, and your 6C hand test confirmed 15360 ticks per turn. While driving it reports ratios around 0.9–0.95, which come from the Pico's speed estimate lagging, not from the ticks.

Leave Pi-1 running.

### 3.3 Battery and odometry, with no motor power

**Pi-2** (do the "Pi terminal start" box first).

Battery voltage:

```bash
ros2 topic echo --once /pico/battery --field voltage
```

**Expected:** **12.3 V or more** after charging (full = 12.6 V). Stop testing for the day at **11.4 V**. The Pico cuts off at 10.5 V.

The `/odom` fix (robot DISARMED, wheels still). This records 12 s of the forward speed:

```bash
timeout 12 ros2 topic echo /odom --field twist.twist.linear.x > ~/odom_v.txt
```

Smallest and largest value:

```bash
grep -v -- --- ~/odom_v.txt | sort -g | sed -n '1p;$p'
```

Number of samples beyond ±0.03 m/s:

```bash
grep -v -- --- ~/odom_v.txt | awk '$1>0.03 || $1<-0.03' | wc -l
```

**Pass:** smallest and largest within about **±0.005**, and **0** samples beyond ±0.03. Before the fix (17 Oct) it was −0.0702 / +0.0313, with 9 samples over.
**If not:** run this and send me the output together with the numbers:

```bash
ros2 param get /pico_bridge twist_from_ticks
```

### 3.4 One short drive on blocks (about 20 s of motor power)

1. Helper at the E-stop. Wheels off the floor.
2. **Blue** switch **on**.

**Pi-2** — arm (the column homes down, ~3 s):

```bash
ros2 service call /pico_bridge/arm std_srvs/srv/SetBool "{data: true}"
```

**Pi-2** — watch the speeds (leave it printing):

```bash
ros2 topic echo /odom --field twist.twist
```

**PC-A** (do the "PC terminal start" box first) — keyboard driving:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -p frame_id:=base_footprint -p speed:=0.15 -p turn:=0.5
```

Click into PC-A so it receives your key presses, then:

| Press | For | Watch in Pi-2 | Pass |
|---|---|---|---|
| `i` (hold) | 3 s | `linear.x` | about **+0.15**; both wheels turn forwards |
| `k` | — | `linear.x` | back to about **0** |
| `j` (hold) — *only if your 6C turn test was before the sign change* | 3 s | `angular.z` | **positive**; looking from above, the left wheel turns backwards and the right wheel forwards |
| `k` | — | | wheels stop |

Then stop teleop: `Ctrl+C` in **PC-A**. Stop the echo: `Ctrl+C` in **Pi-2**.

**Pi-2** — disarm:

```bash
ros2 service call /pico_bridge/arm std_srvs/srv/SetBool "{data: false}"
```

3. **Blue** switch **off**.

**If `j` gives a negative `angular.z`, or the wrong wheel goes backwards:** stop here and tell me. Mapping won't work until it's fixed.

### 3.5 LiDAR points the right way (no motor power)

**PC-B** (do the "PC terminal start" box first) — open RViz with the sensor view from the zip:

```bash
rviz2 -d ~/phase7/ros/hospital_robot_nav/rviz/phase7_sensors.rviz
```

RViz shows a top view of the robot model, a red arrow pointing out of its front, and the LiDAR points in white.

1. Hold a box about 50 cm **in front** of the robot.
2. **Pass:** the white dots of the box appear in front of the model, on the side the red arrow points to.

If the dots appear beside or behind the robot, the LiDAR yaw in the URDF is wrong. Tell me before Stage 5.

**No white dots at all?** Check in this order:

1. **Pi-2** — is the LiDAR publishing? Expect `average rate: ~10` (`Ctrl+C` to stop):

```bash
ros2 topic hz /scan
```

2. If nothing arrives: look in **Pi-1** for `process has died … ldlidar` (see 3.2), restart the base after the clock-sync line.
3. If it still dies: stop the base (`Ctrl+C` in Pi-1), then test the LiDAR on its own in **Pi-1**. Expect ~400 packets/s and spin ~10 Hz:

```bash
ros2 run hospital_robot_base ld06_check
```

4. If `/scan` arrives at ~10 Hz but RViz shows nothing: in RViz's Displays panel, look at the LaserScan display's status line and send it to me.

Close RViz (window ✕) when done.

### 3.6 Leave the base running?

* Continuing straight to Stage 4 → stop the base now: `Ctrl+C` in **Pi-1** (the Pico must be free for flashing).
* Stopping for the day → `Ctrl+C` in **Pi-1**, then the green switch off.

**Pass condition, Stage 3:** battery ≥ 12.0 V, `/odom` still-noise gone, forward = +0.15, (`j` = positive if needed), box in front in RViz.

---

## Stage 4 — Ultrasonics: new Pico firmware, then test each sensor

**Objective:** put firmware 1.2.0 (your current firmware + the ultrasonic code) on the Pico, then check every sensor.

**Where the firmware lives:** the Arduino sketch is **on the PC**, and you flash the Pico **from the PC** with the Arduino IDE, exactly as in Phases 4, 6C and 8. Nothing firmware-related is copied to the Pi. The Pi only needs the bridge patch you already applied in Stage 0.

**The order:**
1. Patch the sketch on the PC.
2. Stop the bridge on the Pi.
3. Move the Pico's USB cable from the Pi to the PC.
4. Upload.
5. Move the cable back to the Pi.
6. Test.

### 4.1 Find your current firmware folder (PC)

**PC-A:**

```bash
grep -rn "define LEFT_FORWARD_SIGN" ~/Arduino ~/robot_ws/src/firmware 2>/dev/null
```

This lists every copy of the firmware on the PC. **Use the folder whose line shows your new sign** (the one you changed on 18 Oct, `(-1)` for the left). Example output:

```
/home/<you>/Arduino/pico_motor_firmware/config.h:102:#define LEFT_FORWARD_SIGN   (-1)
```

→ the folder is `~/Arduino/pico_motor_firmware`.

In the commands below I use `~/Arduino/pico_motor_firmware`. **If yours is different, change that part in every command of Stage 4.**

Check the folder has the three firmware files:

```bash
ls ~/Arduino/pico_motor_firmware
```

**Expected:** `config.h  lift.h  pico_motor_firmware.ino` (maybe more).

### 4.2 Back up the working firmware (PC)

**PC-A:**

```bash
cp -a ~/Arduino/pico_motor_firmware ~/Arduino/pico_motor_firmware_1.1.0_backup
```

This copy is your known-working 1.1.0 with the new wheel signs. If anything goes wrong, you flash this folder.

### 4.3 Add the ultrasonic code to the firmware (PC)

**PC-A** — dry run (shows the changes, writes nothing):

```bash
cd ~/phase7/patches && python3 apply_firmware_patch.py --sketch ~/Arduino/pico_motor_firmware
```

**Expected:** a list of 7 changes and `DRY RUN: nothing was written.`

**PC-A** — apply:

```bash
python3 apply_firmware_patch.py --sketch ~/Arduino/pico_motor_firmware --apply
```

**Expected:** `written: …pico_motor_firmware.ino`, `written: …config.h`, `copied: …ultrasonic.h`.

Check that your wheel signs are untouched, and the new version number:

```bash
grep -n "FORWARD_SIGN\|FIRMWARE_VERSION" ~/Arduino/pico_motor_firmware/config.h
```

**Expected:** your signs exactly as before, and `FIRMWARE_VERSION "1.2.0"`.

### 4.4 Get the robot ready for flashing

1. **Pi-1:** if the base is still running, press `Ctrl+C` and wait until it has stopped.
2. Robot **disarmed**. **Blue** switch **off**.
3. **Green** switch **on** (the sensors need their 5 V, and the Pi stays on).
4. **Unplug the Pico's USB cable from the Pi.** Remember which Pi USB port it was in.
5. **Plug that cable into the Dell.**

**PC-A** — the Pico should appear:

```bash
ls /dev/ttyACM*
```

**Expected:** `/dev/ttyACM0` (or `ttyACM1`).

### 4.5 Upload with the Arduino IDE (PC)

Start the Arduino IDE the way you always do. If you use the AppImage, in **PC-B**:

```bash
~/Downloads/arduino-ide_*.AppImage --no-sandbox
```

In the IDE:
1. **File → Open…** → `~/Arduino/pico_motor_firmware/pico_motor_firmware.ino`. Four tabs must appear: `pico_motor_firmware.ino`, `config.h`, `lift.h`, `ultrasonic.h`.
2. **Tools → Board → Raspberry Pi Pico/RP2040 → Raspberry Pi Pico W**. (The IDE may still have the ESP32 selected from Phase 8: check this.)
3. **Tools → Port → /dev/ttyACM0**.
4. Click **Verify (✓)**. **Expected:** "Done compiling". (I compiled the same changes against your 9 Oct firmware here without errors.)
5. Click **Upload (→)**. **Expected:** the upload finishes without errors.
   * If the port disappears or the upload fails: unplug the USB, hold the Pico's **BOOTSEL** button while plugging it back into the PC, release, then click Upload again.

Optional quick check:
1. Open the Serial Monitor (magnifier icon), set **115200** and **New Line**.
2. Type `i` and press Enter.
3. Untick **Autoscroll** and look for `# pico_motor_firmware 1.2.0`, `# forward signs: L=... R=...` (your signs) and `# ultrasonic: ON, one ping every 60 ms`.
4. **Close the Serial Monitor.**

Save the patched firmware in git (**PC-A**). Only run this if that folder is in a git repository; if it says "not a git repository", skip it:

```bash
cd ~/Arduino/pico_motor_firmware && git add -A && git commit -m "Pico firmware 1.2.0: ultrasonic sensors"
```

### 4.6 Put the Pico back on the Pi

1. Unplug the Pico's USB cable from the Dell.
2. Plug it back into the **same USB port on the Pi** as before.

**PC-A** — log in to the Pi (this becomes **Pi-1**):

```bash
ssh wright@robot.local
```

```bash
source /opt/ros/jazzy/setup.bash && source ~/robot_ws/install/setup.bash
```

Check the Pico is back:

```bash
ls -l /dev/pico
```

**Expected:** `/dev/pico -> ttyACM0` (or similar).

**Pi-1** — start the base:

```bash
ros2 launch hospital_robot_base base.launch.py use_head:=true
```

**Expected** in Pi-1, among the start-up lines:
* `[Pico] pico_motor_firmware 1.2.0`
* `[Pico] forward signs: L=… R=…` — **your** signs. If they are the old ones (`L=1 R=-1`), you flashed the wrong folder: flash `~/Arduino/pico_motor_firmware_1.1.0_backup` and go back to 4.3 with the right folder.
* `[Pico] ultrasonic: ON, one ping every 60 ms, timeout 30000 us`
* three lines `[Pico] US1 left …`, `[Pico] US2 front …`, `[Pico] US3 right …`
* `Ultrasonic: first reading received from the Pico.`

### 4.7 Live view of the three sensors (Pi)

**Pi-2:**

```bash
ros2 run hospital_robot_nav ultrasonic_check
```

Once a second it prints one line like:

```
left : 5.5 Hz last  2.00 m [2.00-2.00] no-echo 5/5 | front: 5.5 Hz last  1.43 m ... | right: ...
```

| What you see | Meaning |
|---|---|
| All three about **5.5 Hz** | good (one ping every 60 ms, three sensors in turn) |
| **2.00 m** and "no-echo" | nothing within 2 m = "clear" (normal in open space) |
| `--- no data ---` for one sensor, and Pi-1 warns `ECHO stuck HIGH` | that sensor's ECHO line is stuck high: wiring, or a faulty sensor |
| Always 2.00 even with your hand in front | sensor not working (no power, TRIG not connected). That's why you test each one by hand next |
| A constant 0.3–0.6 m with nothing in front | probably the **floor** echo: tilt that sensor up a few degrees |

Leave it running for 4.8.

### 4.8 Hand test — which sensor is which (Pi)

Hold a flat board (book, cardboard) **30 cm** in front of one sensor at a time. Watch which column in **Pi-2** drops to about **0.30**. Then do the same at **100 cm**.

Fill in:

| Physical sensor | Column that reacted (left/front/right) | Reading at 30 cm | Reading at 100 cm |
|---|---|---|---|
| Left panel (faces left) | | | |
| Front panel (faces forward) | | | |
| Right panel (faces right) | | | |

**Pass:** each sensor moves its own column, and readings are within **±2 cm**.

**If the columns are swapped** (e.g. the front sensor moves the `left` column), the wiring order differs from the schematic. Fix it in software, not by rewiring.

**Pi-2** — stop the live view with `Ctrl+C`, then open the base settings:

```bash
nano ~/robot_ws/src/hospital_robot_base/config/base.yaml
```

Find this line (about line 34, in the `pico_bridge:` section):

```yaml
    echo_pico_text: true         # show the Pico's '#' messages in the log
```

Directly **below** it, add these two lines (4 spaces at the start, like the line above). Write the topics in the order **US1, US2, US3**, as your hand test showed. Example: US1 is really the front sensor and US2 the left one:

```yaml
    ultrasonic_topics: ["ultrasonic/front", "ultrasonic/left", "ultrasonic/right"]
    ultrasonic_frames: ["us_front_link", "us_left_link", "us_right_link"]
```

Keep each topic paired with its frame (`front` ↔ `us_front_link`). Save: `Ctrl+O`, `Enter`, `Ctrl+X`.

Rebuild and restart:

```bash
cd ~/robot_ws && colcon build --symlink-install --packages-select hospital_robot_base
```

Then in **Pi-1**: `Ctrl+C`, and start the base again:

```bash
ros2 launch hospital_robot_base base.launch.py use_head:=true
```

Repeat 4.7 and 4.8 until every sensor moves its own column.

### 4.9 See the cones in RViz (PC)

**PC-B:**

```bash
rviz2 -d ~/phase7/ros/hospital_robot_nav/rviz/phase7_sensors.rviz
```

You see the robot from above with three coloured cones: orange = left, blue = front, purple = right.

**Pass:**
* Each cone starts at its sensor and points the way that sensor faces.
* Holding the board in front of a sensor makes **its** cone shorter.

A cone in the wrong place or pointing the wrong way = a wrong number in `ultrasonic.xacro` (Stage 2.4).

Close RViz when done.

**Pi-2** — save the work in git:

```bash
cd ~/robot_ws/src && git add -A && git commit -m "Phase 7: ultrasonic sensors working"
```

**Pass condition, Stage 4:** firmware 1.2.0 with your wheel signs, three sensors at ~5.5 Hz, each reacts on its own column, ±2 cm, cones point the right way.

**If one sensor can't be made to work:** you can still continue. In Stages 6–9 add ` use_ultrasonic:=false` to the end of every `navigation.launch.py` command, and note it as a limitation.

---

## Stage 5 — Make the map of your apartment (roadmap 7.1)

**Objective:** a map of your apartment, saved as `~/robot_ws/maps/apartment/map.yaml` + `map.pgm` on the Pi.

**Each place gets its own folder (a "site"):**

| Site | Folder on the Pi | Used with |
|---|---|---|
| `apartment` (now) | `~/robot_ws/maps/apartment/` | `site:=apartment` / `--site apartment` |
| `hospital` (later, for the demo) | `~/robot_ws/maps/hospital/` | `site:=hospital` / `--site hospital` |

The map and the named places of one site never mix with the other.

### 5.1 Prepare the apartment

* Clear the floor along the route: cables, shoes, bags.
* Doors: decide which are open. Keep them the same later, when navigating.
* Close curtains in front of full-length mirrors or glass doors if you can. The LiDAR sees through or into them.
* Nobody walking around during mapping.

### 5.2 Terminals for this stage

| Terminal | Runs |
|---|---|
| Pi-1 | the robot base |
| Pi-2 | SLAM (the mapping program) |
| Pi-3 | arm / disarm / save the map |
| PC-A | keyboard driving |
| PC-B | RViz (watch the map grow) |

### 5.3 Start everything

Robot **on the floor** in an open spot (the hallway near your front door is a good start). **Blue** switch **off**, **green** switch **on**.

**Pi-1:**

```bash
ros2 launch hospital_robot_base base.launch.py use_head:=true
```

Wait for `[head] STARTING -> WAIT_ARM`.

**Pi-2:**

```bash
ros2 launch hospital_robot_nav mapping.launch.py
```

**Expected** in Pi-2:
* `scan_body_filter: removing points inside x[-0.23,+0.23] y[-0.26,+0.26] m of base_footprint.`
* `lidar pose in base_footprint: x=… y=… yaw=-90.0 deg (from the URDF)`
* slam_toolbox lines, and after 30 s: `last 30 s: removed N of M beams (x %) as robot body`. A few % is normal (the mast). Over ~25 %: tell me.

**PC-B:**

```bash
ros2 launch nav2_bringup rviz_launch.py
```

In RViz: the map appears around the robot in black (walls), white (free) and grey (unknown). The **Fixed Frame** (top left, Global Options) must be `map`.

### 5.4 Arm (column homes down)

1. **Blue** switch **on**. Helper near the E-stop.

**Pi-3:**

```bash
ros2 service call /pico_bridge/arm std_srvs/srv/SetBool "{data: true}"
```

**Pi-3** — wait until the column is parked:

```bash
ros2 topic echo /head_column/state
```

When it prints `data: PARKED` over and over, press `Ctrl+C` in Pi-3. **Never drive before `PARKED`** (screen down).

### 5.5 Drive slowly through the apartment

**PC-A:**

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -p frame_id:=base_footprint -p speed:=0.15 -p turn:=0.4
```

Click into PC-A. Keys: `i` forward, `,` back, `j` turn left, `l` turn right, `k` stop. **Don't press `q`, `w` or `e`** (they speed up).

How to drive for a good map:
1. **Slowly.** Turn even more slowly: short taps on `j`/`l`, pausing between them. Fast turns break the map.
2. Drive through every room and corridor you want to use later, along the middle.
3. In each room, stop and turn slowly once all the way round.
4. **Come back past the start point at the end** (this "closes the loop" and straightens the map).
5. Watch PC-B: walls must stay single, sharp lines. If a wall appears twice, slightly rotated, you turned too fast. Drive back over that area slowly.

When the map covers everything: press `k`, then `Ctrl+C` in **PC-A**.

### 5.6 Save the map (Pi)

**Pi-3** — make the site folder:

```bash
mkdir -p ~/robot_ws/maps/apartment
```

**Pi-3** — save the map image + description:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/robot_ws/maps/apartment/map --ros-args -p map_subscribe_transient_local:=true
```

**Expected:** `Map saved successfully`.

**Pi-3** — also save SLAM's internal map, so you can extend it later instead of starting again:

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$HOME/robot_ws/maps/apartment/posegraph'}"
```

**Pi-3** — check the files:

```bash
ls ~/robot_ws/maps/apartment
```

**Expected:** `map.pgm  map.yaml  posegraph.data  posegraph.posegraph`.

### 5.7 Disarm and stop mapping

**Pi-3:**

```bash
ros2 service call /pico_bridge/arm std_srvs/srv/SetBool "{data: false}"
```

**Blue** switch **off**.

**Pi-2:** `Ctrl+C` — **mapping must be stopped before navigation is ever started** (both would control the map).
**PC-B:** close RViz.
**Pi-1:** leave the base running if you go on to Stage 6.

### 5.8 Look at the map on the PC

**PC-A:**

```bash
scp wright@robot.local:~/robot_ws/maps/apartment/map.pgm ~/apartment_map.pgm
```

```bash
xdg-open ~/apartment_map.pgm
```

**Pass condition, Stage 5:** the image looks like your apartment's floor plan; walls single and straight; no grey holes along the routes you'll drive. **Not good?** Repeat 5.3–5.7. Saving again simply overwrites the files.

---

## Stage 6 — The named places (waypoints)

**Objective:** save four places in `~/robot_ws/maps/apartment/waypoints.yaml`.

**The names are fixed.** They are what Anna's touchscreen buttons send. In your apartment you choose which real spot each one is:

| Name (type exactly) | Touchscreen button | Example in the apartment (you choose) |
|---|---|---|
| `home` | — (start and return point = "reception") | in the hallway, near the front door |
| `doctor` | Doctor's office | bedroom door |
| `xray` | X-ray room | bathroom door |
| `pharmacy` | Pharmacy | kitchen |

The `--label` text is only for your own notes in the file. The screen always shows Anna's button names.

Pick spots with **at least 60 cm free all round**, reachable without squeezing through narrow gaps.

### 6.1 Terminals for this stage

| Terminal | Runs |
|---|---|
| Pi-1 | the robot base (still running from Stage 5, or start it again) |
| Pi-2 | navigation (localisation on your map) |
| Pi-3 | arm / disarm / save the places |
| PC-A | keyboard driving |
| PC-B | RViz |

### 6.2 Put the robot at "home"

1. Place the robot where `home` should be, facing the direction it should start in (e.g. facing into the apartment).
2. Mark the spot with masking tape: one strip along the outside of each drive wheel, and one across the front edge. You'll put the robot back on these marks before every session.

### 6.3 Start navigation

If Pi-1 isn't running the base any more, **Pi-1:**

```bash
ros2 launch hospital_robot_base base.launch.py use_head:=true
```

**Pi-2:**

```bash
ros2 launch hospital_robot_nav navigation.launch.py site:=apartment
```

**Expected** in Pi-2:
* `[navigation] SITE: apartment`
* `[navigation] map: /home/wright/robot_ws/maps/apartment/map.yaml`
* `[navigation] no "home" in …waypoints.yaml: set the pose with RViz 2D Pose Estimate` (normal the first time)
* after 10–20 s: `Managed nodes are active`, **twice**
* `mission: … waypoint file not found …` and `mission ready` (normal the first time)

**PC-B:**

```bash
ros2 launch nav2_bringup rviz_launch.py
```

### 6.4 Tell the robot where it is (first time only)

In RViz (PC-B):
1. Click **2D Pose Estimate** (top toolbar).
2. Click on the map **where the robot really stands**, hold the mouse button, drag in the direction the robot faces, release.
3. **Check:** the LiDAR points (coloured dots) must now lie on the black walls of the map. If not, repeat steps 1–2 more carefully.

### 6.5 Save `home`

**Pi-3** (robot still on the tape marks):

```bash
ros2 run hospital_robot_nav save_waypoint home --site apartment --label "Front door"
```

**Expected:** `saved "home": x=… y=… yaw=… deg -> ~/robot_ws/maps/apartment/waypoints.yaml`.

### 6.6 Arm, drive to each place, save it

1. **Blue** switch **on**. Helper near the E-stop.

**Pi-3:**

```bash
ros2 service call /pico_bridge/arm std_srvs/srv/SetBool "{data: true}"
```

**Pi-3** — wait for `data: PARKED`, then `Ctrl+C`:

```bash
ros2 topic echo /head_column/state
```

**PC-A:**

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -p frame_id:=base_footprint -p speed:=0.15 -p turn:=0.4
```

For each of the three places:
1. Drive to the spot (PC-A).
2. Turn so the robot faces the way it should stand when it arrives there with a patient. Press `k`.
3. In RViz (PC-B), check the LiDAR dots still lie on the walls.
4. Save it in **Pi-3** with the matching command below.

```bash
ros2 run hospital_robot_nav save_waypoint doctor --site apartment --label "Bedroom door"
```

```bash
ros2 run hospital_robot_nav save_waypoint xray --site apartment --label "Bathroom door"
```

```bash
ros2 run hospital_robot_nav save_waypoint pharmacy --site apartment --label "Kitchen"
```

(Change the labels to whatever you chose. Saving a name again replaces it.)

Drive back onto the tape marks at `home`, press `k`, then `Ctrl+C` in **PC-A**.

**Pi-3** — check all four are there:

```bash
ros2 run hospital_robot_nav save_waypoint --list --site apartment
```

**Pi-3** — disarm:

```bash
ros2 service call /pico_bridge/arm std_srvs/srv/SetBool "{data: false}"
```

**Blue** switch **off**.

### 6.7 Check that the start position works by itself

**Pi-2:** `Ctrl+C`, wait until it has stopped, then start it again:

```bash
ros2 launch hospital_robot_nav navigation.launch.py site:=apartment
```

**Expected** now: `[navigation] AMCL initial pose = home x=… y=… yaw=… deg`.

**PC-B:** close RViz and open it again:

```bash
ros2 launch nav2_bringup rviz_launch.py
```

**Pass condition, Stage 6:** `--list` shows `home`, `doctor`, `xray`, `pharmacy`. After the restart, with the robot on the tape marks, the LiDAR dots lie on the walls **without** using 2D Pose Estimate.

Leave Pi-1, Pi-2 and PC-B running for Stage 7.

---

## Stage 7 — Navigation tests (roadmap 7.2, 7.3)

**Terminals:** Pi-1 base, Pi-2 navigation, PC-B RViz (all running from Stage 6), Pi-3 for commands.

### 7.1 Configuration checks (robot disarmed)

**Pi-3** — the controller sends the right message type (expect `True`):

```bash
ros2 param get /controller_server enable_stamped_cmd_vel
```

**Pi-3** — the guiding speed (expect `0.25`):

```bash
ros2 param get /controller_server FollowPath.desired_linear_vel
```

**Pi-3** — who sends motor commands:

```bash
ros2 topic info /cmd_vel -v
```

**Expected:**
* Type: `geometry_msgs/msg/TwistStamped`.
* Publisher: `collision_monitor`.
* Subscriber: `pico_bridge`.

If the type is `Twist`, the robot won't move: send me the output.

**Any errors when Nav2 started?** To check, restart navigation with a log file.

1. **Pi-2:** press `Ctrl+C` and wait until it has stopped.
2. **Pi-2** — start it again, this time saving everything it prints:

```bash
ros2 launch hospital_robot_nav navigation.launch.py site:=apartment 2>&1 | tee ~/nav_test.log
```

3. Wait for `Managed nodes are active` twice.
4. **Pi-3** — search the log for problems:

```bash
grep -nE "ERROR|Failed|not declared|exception" ~/nav_test.log | head -30
```

**Expected:** nothing, or only lines you can match in the Troubleshooting table. **Send me anything else.** This is the first time these settings meet your installed Nav2.

### 7.2 Sensors in the obstacle map (robot disarmed)

In RViz (PC-B), find **Controller → Local Costmap** in the Displays list on the left and tick it on if it isn't.

1. Hold a board **30 cm** in front of the **front ultrasonic** (low, below the LiDAR's height).
2. **Pass:** a coloured patch appears in front of the robot in the local costmap within about 1 s.
3. Take the board away → the patch disappears within about 1–2 s.

**Pi-3** — the emergency-stop layer's state (leave it running for 7.3):

```bash
ros2 topic echo /collision_monitor_state
```

### 7.3 First goal on blocks (wheels off the floor)

1. Put the robot **on blocks**, still on the tape marks (localisation stays fine because it doesn't move).
2. **Blue** switch **on**. Helper near the E-stop.

**Pi-4** (do the "Pi terminal start" box first) — arm:

```bash
ros2 service call /pico_bridge/arm std_srvs/srv/SetBool "{data: true}"
```

Wait ~5 s for the column to park.

In RViz (PC-B):
1. Click **Nav2 Goal** (top toolbar).
2. Click on the map **1 m in front of the robot**, drag in the forward direction, release.

| Check | Pass |
|---|---|
| Wheels | turn **forwards**, slowly and steadily |
| Hand ~15 cm in front of the front edge | wheels **stop** within a fraction of a second; Pi-3 shows a stop action in `collision_monitor_state` |
| Hand away | wheels start again |
| RViz → Navigation 2 panel (bottom left) → **Cancel** | wheels stop |

(On blocks the robot never "arrives": cancelling is normal.)

**Pi-4** — disarm:

```bash
ros2 service call /pico_bridge/arm std_srvs/srv/SetBool "{data: false}"
```

**Blue** switch **off**. Take the robot off the blocks and put it back on the tape marks.

### 7.4 On the floor

**Pi-2:** `Ctrl+C`, then start again so AMCL takes the home pose:

```bash
ros2 launch hospital_robot_nav navigation.launch.py site:=apartment
```

1. **Blue** switch **on**.

**Pi-4:**

```bash
ros2 service call /pico_bridge/arm std_srvs/srv/SetBool "{data: true}"
```

Do each test with **Nav2 Goal** in RViz. Write down the result:

| # | Test | Pass |
|---|---|---|
| 1 | Goal 1–2 m straight ahead | smooth drive, slows near the end, stops within ~15 cm, turns to the arrow's direction |
| 2 | Goal **behind** the robot | turns on the spot first, then drives. **Watch for a STALL fault** (below) |
| 3 | Goal into the next room | goes through the doorway without touching the frame |
| 4 | Someone steps into the path 2 m ahead | slows and stops, or goes around; never touches |
| 5 | Shoebox (lower than the LiDAR) in the path | stops or goes around (ultrasonics working) |
| 6 | Test 5 again with ultrasonics off (see below) | the robot hits / nearly hits the low box → shows why the ultrasonics matter (good evidence for the report). **Helper ready on the E-stop** |
| 7 | Chair with thin legs, mirror or glass door (if you have them) | write down what happens: roadmap 7.3 limitations |

**STALL fault in test 2:** turning on the spot at 0.6 rad/s is only ~1.0 rad/s at the wheels (0.6 × 0.245/2 ÷ 0.072), right at the Pico's `STALL_MIN_CMD` (1.0 rad/s). If the casters scrub and the robot stops with `FAULT`, run in **Pi-4**:

```bash
ros2 topic echo --once /pico/status
```

If it shows `STALL_L*` or `STALL_R*`, tell me. I'll raise the turning speed in `nav2_params_real.yaml`. Clear the fault with:

```bash
ros2 service call /pico_bridge/clear_faults std_srvs/srv/Trigger
```

**Test 6 — navigation without the ultrasonics.** In **Pi-2**: `Ctrl+C`, then:

```bash
ros2 launch hospital_robot_nav navigation.launch.py site:=apartment use_ultrasonic:=false
```

After test 6, **Pi-2**: `Ctrl+C` and start normally again:

```bash
ros2 launch hospital_robot_nav navigation.launch.py site:=apartment
```

### 7.5 Is the Pi fast enough?

During a goal, in **Pi-4**:

```bash
top
```

Look at the `%CPU` column for a few seconds, then press `q`. Also watch **Pi-2** for `Control loop missed its desired rate of 10.0000Hz`. Now and then is fine. If it appears constantly, tell me: Nav2 can then run on the Dell instead.

**Finish Stage 7 — Pi-4:**

```bash
ros2 service call /pico_bridge/arm std_srvs/srv/SetBool "{data: false}"
```

**Blue** switch **off**. **Pi-3:** `Ctrl+C` (stops the `collision_monitor_state` echo).

**Pass condition, Stage 7:** `/cmd_vel` is `TwistStamped`, the hand stops the wheels, tests 1–5 pass without touching anything.

---

## Stage 8 — Missions: drive to the named places

**Objective:** the mission node drives to `home`/`doctor`/`xray`/`pharmacy` by name, and refuses to drive when it isn't safe.

**Terminals:** Pi-1 base, Pi-2 navigation (both running), Pi-3 commands, PC-B RViz (optional, to watch).

Robot on the `home` tape marks. If you restarted anything, check **Pi-2** printed `AMCL initial pose = home`.

### 8.1 Arm

1. **Blue** switch **on**. Helper near the E-stop.

**Pi-3:**

```bash
ros2 service call /pico_bridge/arm std_srvs/srv/SetBool "{data: true}"
```

**Pi-3** — wait for `data: PARKED`, then `Ctrl+C`:

```bash
ros2 topic echo /head_column/state
```

### 8.2 Drive to each place

`go_to` prints the state and the remaining distance until it finishes. **`Ctrl+C` in its terminal stops the robot.**

**Pi-3** — guiding speed (0.25 m/s):

```bash
ros2 run hospital_robot_nav go_to doctor
```

**Expected:** `STARTING` → `ACTIVE` with the remaining distance going down → `SUCCEEDED`.

**Pi-3** — back home at "alone" speed (0.35 m/s):

```bash
ros2 run hospital_robot_nav go_to home --mode alone
```

**Pi-3:**

```bash
ros2 run hospital_robot_nav go_to xray
```

```bash
ros2 run hospital_robot_nav go_to home --mode alone
```

```bash
ros2 run hospital_robot_nav go_to pharmacy
```

```bash
ros2 run hospital_robot_nav go_to home --mode alone
```

After each arrival, measure how far the robot is from where you saved that place (for the report).

### 8.3 Safety tests

Each test must **stop the robot** or **refuse to start**. Do them one at a time, from `home`.

**Test 1 — refuses while disarmed.** **Pi-3**, disarm:

```bash
ros2 service call /pico_bridge/arm std_srvs/srv/SetBool "{data: false}"
```

```bash
ros2 run hospital_robot_nav go_to xray
```

**Pass:** `FAILED … not_armed`, nothing moves. Arm again:

```bash
ros2 service call /pico_bridge/arm std_srvs/srv/SetBool "{data: true}"
```

Wait for the column to park (~5 s).

**Test 2 — unknown place.** **Pi-3:**

```bash
ros2 run hospital_robot_nav go_to canteen
```

**Pass:** `FAILED … unknown_target`, nothing moves.

**Test 3 — cancel.** **Pi-3:**

```bash
ros2 run hospital_robot_nav go_to pharmacy
```

After 2 seconds of driving press `Ctrl+C` in Pi-3. **Pass:** `cancel sent`, the robot stops.

**Test 4 — column leaves PARKED.** **Pi-3:**

```bash
ros2 run hospital_robot_nav go_to pharmacy
```

While it drives, in **Pi-4** (re-homes the column; it is already at the bottom, so it barely moves):

```bash
ros2 topic pub --once /head_column/command std_msgs/msg/String "{data: home}"
```

**Pass:** Pi-3 shows `FAILED … head_not_parked`, the robot stops.

**Test 5 — E-stop.** **Pi-3:**

```bash
ros2 run hospital_robot_nav go_to pharmacy
```

While it drives, press the **E-stop**. **Pass:** the robot stops at once; Pi-3 shows `FAILED … motors_off`. Release the E-stop (it stays disarmed). **Pi-4:**

```bash
ros2 service call /pico_bridge/arm std_srvs/srv/SetBool "{data: true}"
```

**Pi-3** — drive back:

```bash
ros2 run hospital_robot_nav go_to home --mode alone
```

### 8.4 Finish

**Pi-3:**

```bash
ros2 service call /pico_bridge/arm std_srvs/srv/SetBool "{data: false}"
```

**Blue** switch **off**.

**Pass condition, Stage 8:** home → each place → home, three times without help; all five safety tests as described.

---

## Stage 9 — The full demo (and the video)

**Objective:** Anna welcomes a person, asks where they want to go, guides them, offers to wait, and returns with them or alone.

### 9.1 Before you start

* Battery **≥ 12.0 V** (check in 9.3).
* Robot **on the `home` tape marks**, wall adapter **unplugged**.
* **Green** switch **on**, **blue** switch **off** for now.
* A helper at the E-stop for every run.

### 9.2 Terminals

| Terminal | Runs |
|---|---|
| Pi-1 | robot base + head column |
| Pi-2 | navigation |
| Pi-3 | Anna (eyes, voice, touchscreen flow) |
| Pi-4 | screen restart, arming, recording, staff commands |
| PC-B | RViz (optional, nice for a split-screen video) |

### 9.3 Start-up, in this order

**Pi-1:**

```bash
ros2 launch hospital_robot_base base.launch.py use_head:=true 2>&1 | tee ~/demo_pi1.log
```

Wait for `[head] STARTING -> WAIT_ARM` and `Connected to the head camera on /dev/headcam`.

**Pi-2:**

```bash
ros2 launch hospital_robot_nav navigation.launch.py site:=apartment 2>&1 | tee ~/demo_pi2.log
```

Wait for `AMCL initial pose = home`, `Managed nodes are active` (twice) and `mission ready`.

**Pi-3:**

```bash
ros2 launch hospital_robot_hri hri.launch.py use_gaze:=true nav:=true 2>&1 | tee ~/demo_pi3.log
```

Wait for `hri_interaction: … navigation ON`.

**Pi-4** — restart the robot's screen (only after Pi-3 runs):

```bash
sudo snap restart wpe-webkit-mir-kiosk
```

**Pi-4** — check the programs are running:

```bash
ros2 node list | grep -E "hri|mission|bt_navigator|amcl|head_column"
```

**Expected:** `/amcl`, `/bt_navigator`, `/head_column`, `/hri_face`, `/hri_interaction`, `/hri_speech`, `/mission`.

**Pi-4** — battery:

```bash
ros2 topic echo --once /pico/battery --field voltage
```

**PC-B** (optional):

```bash
ros2 launch nav2_bringup rviz_launch.py
```

**Arm:** everyone clear, helper at the E-stop, **blue** switch **on**.

**Pi-4:**

```bash
ros2 service call /pico_bridge/arm std_srvs/srv/SetBool "{data: true}"
```

Watch Pi-1 for `[head] … -> PARKED`. Anna says "My motors are now on…". The screen shows the sleeping eyes and "Tap the screen to wake me".

### 9.4 Record evidence while you film

**Pi-4** (stop it with `Ctrl+C` after the last take):

```bash
ros2 bag record -o ~/demo_bag_$(date +%Y%m%d_%H%M) /mission/status /hri/interaction/state /robot_state /odom /cmd_vel /collision_monitor_state /ultrasonic/front /amcl_pose
```

### 9.5 The takes

Each take starts with the robot asleep at `home`. Takes A–C bring it back home by themselves.

| Take | You do | You should see / hear |
|---|---|---|
| **A — wait, return together** | Tap the screen → wait for the height adjustment → **Pharmacy** → walk behind it → at arrival **Wait for me here** → wait a moment → **Take me back to reception** → follow it home | intro → column rises to your face → "The pharmacy…" → column lowers → "Please follow me…" → drives at walking pace → "We have arrived…" → "I will wait for you right here…" → "Let's go back to reception together…" → "We are back at reception…" → sleeps |
| **B — goes back alone** | Tap → **Doctor's office** → at arrival **Go back without me** | "I'm heading back to my base now…" → drives home alone (a bit faster) → sleeps |
| **C — nobody answers** | Tap → **X-ray room** → at arrival walk away | after 45 s it goes home alone |
| **D — Stop button** | During a trip press the red **Stop** on the screen → **Continue to …** | stops, "I have stopped…", then carries on |
| **E — obstacle** | During a trip someone steps in front | slows / stops, continues when the way is clear |

### 9.6 Staff commands (if something goes wrong between takes)

**Pi-4** — send the robot home alone (from sleep or from any trip menu):

```bash
ros2 topic pub --once /hri/interaction/command std_msgs/msg/String "{data: home}"
```

**Pi-4** — cancel the trip and put Anna to sleep where she is:

```bash
ros2 topic pub --once /hri/interaction/command std_msgs/msg/String "{data: sleep}"
```

**Pi-4** — what Anna is doing right now (`Ctrl+C` to stop):

```bash
ros2 topic echo /hri/interaction/state
```

**Pi-4** — what the mission node is doing (`Ctrl+C` to stop):

```bash
ros2 topic echo /mission/status
```

### 9.7 Shut down

**Pi-4** — column down, then disarm:

```bash
ros2 topic pub --once /hri/interaction/command std_msgs/msg/String "{data: sleep}"
```

Wait ~5 s, then:

```bash
ros2 service call /pico_bridge/arm std_srvs/srv/SetBool "{data: false}"
```

**Blue** switch **off**. Then `Ctrl+C` in **Pi-4** (the bag recording), **Pi-3**, **Pi-2**, **Pi-1** in that order.

**Pi-4** — before switching the Pi off, shut it down properly (protects the SD card):

```bash
sudo shutdown now
```

Wait for the Pi's green LED to stop flashing, then **green** switch **off**.

### 9.8 Later: the hospital

Do Stages 5 and 6 again at the hospital, with `hospital` instead of `apartment` everywhere:

```bash
mkdir -p ~/robot_ws/maps/hospital
```

```bash
ros2 run nav2_map_server map_saver_cli -f ~/robot_ws/maps/hospital/map --ros-args -p map_subscribe_transient_local:=true
```

```bash
ros2 run hospital_robot_nav save_waypoint home --site hospital --label "Reception"
```

```bash
ros2 launch hospital_robot_nav navigation.launch.py site:=hospital
```

The apartment files stay untouched.

---

## Troubleshooting

| Symptom | Likely cause → what to run / check |
|---|---|
| No `/scan`, Pi-1 shows `ldlidar communication is abnormal` / `process has died` | Clock not synced yet, or the respawn fix missing: Stage 3.2 |
| `ssh: Could not resolve hostname robot.local` | Pi still booting (wait 1 min), or use its IP address: `ssh wright@<ip>` |
| PC terminal doesn't see Pi topics (`ros2 topic list` on the PC has no `/scan`) | `echo $ROS_DOMAIN_ID` differs between PC and Pi; or the PC isn't on the same WiFi |
| Nav2 plans, robot doesn't move | not armed; `/cmd_vel` type (Stage 7.1); collision monitor holding a stop (`ros2 topic echo /collision_monitor_state`) |
| Robot stops after a few seconds and the goal fails, log says costmap/sensor data out of date | an ultrasonic stopped sending. Run `ros2 run hospital_robot_nav ultrasonic_check`. To keep going without them: `use_ultrasonic:=false` |
| Collision monitor stops all the time with nothing in front | a sensor sees the floor or the robot: check the cones in the Stage 4.9 RViz view; floor echo (4.7 table) |
| "Start occupied" / no path at the very start | robot body still in the scan: tell me the `removed N of M beams` line from the navigation log |
| Robot jumps around on the map | mapping still running together with navigation: `Ctrl+C` the mapping terminal |
| Localisation wrong at the start | robot not on the tape marks / facing differently. Use **2D Pose Estimate** in RViz |
| STALL fault while turning on the spot | Stage 7.4 test 2 |
| `go_to` / Anna: `FAILED head_not_parked` | `ros2 topic echo /head_column/state` must say `PARKED`. `STARTING`/`WAIT_ARM` = not armed yet |
| Anna says "I can't reach the destination" right away | `ros2 topic echo /mission/status` → read `reason`. "mission node did not answer" = navigation not running |
| Anna never drives, says "My navigation is not switched on yet" | Pi-3 was started without `nav:=true` |
| A sentence missing / `unknown event` in the Pi-3 log | `ros2 run hospital_robot_hri audio_tool render` (Stage 0.8) |
| Robot goes dead mid-run | battery at 10.5 V (`BATT_CUTOFF`). Charge |
| Screen says "I'm not ready yet" | Pi-3 not running or restarted → `sudo snap restart wpe-webkit-mir-kiosk` after Pi-3 runs |

---

## Rollback

* Any patched file: copy its `.bak_phase7_<time>` back over it (or `git checkout -- <file>`), then rebuild.
* Firmware: flash `~/Arduino/pico_motor_firmware_1.1.0_backup` (Stage 4.5 steps). The bridge simply sees no `u` lines.
* Anna without navigation: start `hri.launch.py` without `nav:=true` — exactly the Phase 9 behaviour.
* `/odom` back to the Pico's own speeds: add `twist_from_ticks: false` under `pico_bridge:` → `ros__parameters:` in `base.yaml`.
* Navigation: just don't launch `hospital_robot_nav`. It changes nothing else.

---

## Files changed on the robot (for your logbook)

| File | Change |
|---|---|
| PC: `~/Arduino/pico_motor_firmware/` (your folder) | + `ultrasonic.h`; `.ino` small edits; `config.h` version 1.2.0 |
| `hospital_robot_base/hospital_robot_base/pico_bridge.py` | ultrasonic hook (3 lines), `twist_from_ticks` |
| `hospital_robot_base/hospital_robot_base/ultrasonic.py`, `test/test_ultrasonic.py` | new |
| `hospital_robot_base/config/base.yaml` | only if the hand test showed swapped sensors (Stage 4.8) |
| `hospital_robot_description/urdf/robot.urdf.xacro` | 1 include line; + `ultrasonic.xacro` with your measurements |
| `hospital_robot_hri/hospital_robot_hri/interaction_node.py` | NavInteractionLogic, `/mission/*` |
| `hospital_robot_hri/hospital_robot_hri/face_logic.py` | passes title/cancel/stop to the page |
| `hospital_robot_hri/web/index.html` | menu titles, 2/3 columns, icons, trip bar + Stop; fixes "No thanks" staying disabled after one use |
| `hospital_robot_hri/config/phrases.yaml` | 5 new events, `arrived` reworded |
| `hospital_robot_hri/hospital_robot_hri/speech_node.py` | `returning_with_you` added to `JOURNEY_EVENTS` (one line) |
| `hospital_robot_hri/launch/hri.launch.py` | `nav:=` argument |
| `hospital_robot_hri/hospital_robot_hri/nav_flow.py`, `test/test_nav_flow.py` | new |
| `hospital_robot_nav/` | new package |
| `~/robot_ws/maps/apartment/` | map, pose graph, waypoints |

---

## Evidence to collect for the report

1. `/odom` still-noise before/after (Stage 3.3 numbers next to the 17 Oct ones).
2. Ultrasonic table (Stage 4.8) and accuracy at 30/100 cm.
3. Map image (`apartment_map.pgm`) + description of the area.
4. Waypoint accuracy: distance from the robot to each saved place after `go_to` (Stage 8.2; 5 runs each → mean and max).
5. Collision-monitor stop distance with a hand/box at 0.25 m/s.
6. Low-obstacle test with and without ultrasonics (Stage 7.4 tests 5 and 6).
7. Full demo: success rate over N runs; any failures and their `/mission/status` reason.
8. Known limitations observed (glass, mirrors, chair legs) — roadmap 7.3.
9. CPU load on the Pi during navigation (Stage 7.5).

---

## Decisions still open (tell me when you get there)

1. **EKF (IMU fusion) for navigation?** This guide uses wheel odometry alone (simpler; the speed fix makes it usable). If localisation struggles in long corridors, we can try `use_imu:=true use_ekf:=true` on the base and compare. Change one thing at a time.
2. **Where Nav2 runs** — Pi first; the Dell if the Pi is overloaded (Stage 7.5).
3. **LiDAR blind sector** (Phase 6C B5-4): if your measurement showed a large blocked angle, give me the angles.
