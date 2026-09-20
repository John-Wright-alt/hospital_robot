# Phase 2 — where you actually are, and exactly what's left

This package already has `robot.urdf.xacro` filled in with real numbers from
your `Measurements.md` and `Parts.md`. You do **not** need to write any xacro
yourself. What's left is: (A) verify four numbers I had to infer, (B) get
mesh files in, (C) build it and look at it. Read this whole file once before
doing anything — it's short.

---

## A. Four things to verify (5–10 minutes with the real robot / Inventor)

Your original walkthrough assumed a tidy setup (one caster, origin at the
drive axle). Your actual measurements show your Inventor origin sits at a
**corner of the chassis footprint**, not the centre — I could tell because
`wheel_x` ≈ half your `base_length`, and `column_y`, `imu_y`, `lidar_y` all
cluster around half your `base_width`. That's fine, the xacro handles it,
but four numbers couldn't come from that corner measurement alone:

1. **`origin_ground_clearance` (currently `0.02` m).** This is the height of
   your Z=0 point above the ground. Your `imu_z` and `lidar_z` values are
   close to or above `base_height`, which suggests your origin is at the
   *bottom face* of the chassis, sitting close to the ground — not at the
   wheel axle. I put a small placeholder in. **You don't need to solve this
   algebraically** — build the package (Part C below), launch it in RViz, and
   look: if the wheels float above the grid, increase this number; if they
   sink into the grid, decrease it. That's the acceptance test your original
   doc already describes in Part 6.4.

2. **`lidar_yaw` (currently `0`).** Your worksheet says "either 90 or 0" —
   meaning you weren't sure if the LiDAR's own 0° mark points toward the
   robot's front. To settle it for real: once the LiDAR is running and
   publishing `/scan`, place a single object directly in front of the robot
   and run `ros2 topic echo /scan --field ranges` (or watch the LaserScan in
   RViz) — the short-range reading should appear in the middle of the
   forward-facing arc. If it instead appears 90° off to one side, change
   `lidar_yaw` to `${pi/2}` or `${-pi/2}`.

3. **`column_length` (currently `0.677971` m) and `camera_x/y/z`.** Your
   worksheet asked for the camera position *relative to the top of the
   column*, but the numbers you gave (`camera_z = 0.764942`) are far too
   large for that — a camera bracket standoff is a few centimetres, not
   76 cm. I'm confident you actually measured the camera relative to
   `base_link` instead, because `camera_y` (0.206) exactly equals
   `column_y` (0.206) — i.e. the camera has zero sideways offset from the
   column, which only makes sense if both numbers share the same reference
   point. I back-calculated: `column_length = camera_z − column_z =
   0.764942 − 0.086971 = 0.677971 m`, and `camera_x = 0.186737 − 0.175 =
   0.011737 m`. This assumes your camera bracket sits right at the top of
   the column with ~0 standoff. **Double check in Inventor**: measure the
   column's own body length when fully retracted (its own top face height
   above its own base mount point) and compare to 0.677971 m. If it's off,
   update `column_length` directly — everything else follows from it.

4. **Masses (`base_mass`, `wheel_mass`, `column_mass`, `camera_mass`,
   `lidar_mass`).** See Part B — these are estimates from your BOM, not
   Inventor, because no material is assigned yet.

None of these four block you from building and viewing the robot today —
they only affect how physically accurate the simulation is later.

---

## B. You have no material assigned — use your Parts.md instead

Your original walkthrough's Part 1.4 wants mass + inertia from Inventor's
**iProperties → Physical** tab, which needs a material assigned to be
meaningful. Skip that entirely for now — it explicitly says to use
placeholder inertia macros when a part isn't ready, and that's exactly what
this package does (`inertial_macros.xacro`, unchanged from the template).

Instead of Inventor mass, I pulled masses straight from your `Parts.md` BOM
and allocated them to links like this:

| Link | Mass used | Where it came from |
|---|---|---|
| `wheel_link` (each) | 0.3 kg | "Scoot/Scate Wheel" (0.235 kg) + motor shaft + bearing share, rounded up |
| `caster_link` (each, ×4) | 0.1 kg | "Tente Swivel Castor Wheel" |
| `lidar_link` | 0.19 kg | Your BOM lists 0.042 kg for the LIDAR line, which is very light for an LD06 unit (typically ~0.19 kg) — I used the realistic figure. Check your actual unit's datasheet and correct if needed. |
| `column_link` | 1.5 kg | NEMA17 stepper + leadscrew + nut + coupling (~0.41 kg) plus an allowance for the column's structural housing, which isn't itemised separately in your BOM |
| `camera_link` | 0.05 kg | FireBeetle ESP32-S3 camera board + a small bracket allowance |
| `base_link` | 11.0 kg | Your BOM's total weight (13.54 kg) minus everything above |

**These are placeholders, same as your original doc recommends.** They make
the physics numerically sane (nothing has zero or infinite mass) without
blocking you. Once you assign real materials in Inventor later — which you
should still do before Phase 3 (Gazebo) — swap these numbers out and the
`<inertial>` blocks will recompute automatically since they're formulas, not
hardcoded tensors.

**When you do get to assigning materials properly**, the steps from your
original walkthrough are:
1. Select the part or sub-assembly → right-click → **iProperties** → **Physical** tab.
2. Set the **Material** dropdown to something real (e.g. Aluminum 6061, ABS Plastic) — this is the step you skipped, and it's why mass reads as meaningless right now.
3. Click **Update**.
4. Read **Mass** and, if you want more accuracy than the placeholder macros give you, expand **Moments of Inertia** and use the values about the **Centre of Gravity** (not the origin). Convert `kg·mm²` to `kg·m²` by dividing by 1,000,000. Set `ixy`, `ixz`, `iyz` to 0 unless the part is dramatically lopsided.

---

## C. Getting meshes out, given your assembly isn't finished

You said you only have two sub-assemblies built (the lifting column, and the
wheel + axle + pulley assembly) — everything else is loose parts in the top
assembly. **You do not need a finished, fully organised assembly to get
STLs.** Your `Robot.stp` already shows every part placed in one flat
top-level assembly (I can see named components in it like `Wheels`,
`Caster`, `LD06_LiDAR`, `Camera plate`, plus your PCBs and structural parts)
— that's enough. Two options, both from your original walkthrough's Part 2.3:

**Fastest — suppress-and-export, no reorganising needed:**
1. Open your top-level assembly in Inventor.
2. In the model browser, right-click and **Suppress** every component
   *except* the ones you want in a given mesh (e.g. suppress everything
   except the chassis plates/brackets/battery box for `base_link.stl`).
3. **File → Export → CAD Format → STL**, with the options from your
   original doc: Binary, Millimeter, Medium resolution, "One file" if
   exporting more than one part.
4. **Unsuppress everything** (right-click → Unsuppress All) before you move
   on to the next mesh, so you don't lose track of what's suppressed.
5. Repeat for `wheel.stl` (you can export just one wheel from your existing
   wheel/axle sub-assembly and reuse it mirrored for the other side),
   `column.stl` (you already have this as a sub-assembly — just open it and
   export directly), and optionally `caster.stl`, `lidar.stl`, `camera.stl`
   (a sphere/cylinder/box placeholder is fine for these, per your original
   doc — they don't need to be pretty).

**If you don't have time to export a mesh at all**, the xacro already falls
back gracefully: every link has a `<collision>` box/cylinder/sphere
regardless of whether you have a mesh. To skip a mesh, delete its `<mesh
.../>` line inside that link's `<visual>` block — RViz will just show the
collision shape instead. This is explicitly fine for Phase 2; don't let a
missing mesh block you.

**One critical thing to get right per part (your original doc's Part 2.1):**
STL export writes vertices relative to the *part's own origin*. The wheel
STL's origin must be at the axle centre, or the wheel will orbit instead of
spin when you move the joint slider. If you didn't set that up in Inventor,
it's faster to just measure the offset and add it to the `<origin xyz="...">`
inside that link's `<visual>` tag in the xacro — every mesh tag already has
one for exactly this reason.

Copy your finished STLs into `meshes/` in this package (replacing the
placeholder text file that's there now), named exactly `base_link.stl`,
`wheel.stl`, `column.stl`, etc. — lowercase, no spaces, matching what's
already written into `robot.urdf.xacro`.

---

## D. Move it to Ubuntu, build it, look at it

This part of your original walkthrough doesn't need any customisation —
follow it exactly:

1. **Get the files onto Ubuntu** — USB stick, or mount the Windows partition
   directly (your original doc's Part 3.1 / 3.2). Put this whole
   `hospital_robot_description` folder inside `~/robot_ws/src/`.

2. **Install tools** (once):
   ```bash
   sudo apt install -y ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui ros-jazzy-robot-state-publisher ros-jazzy-tf2-tools
   ```

3. **Build:**
   ```bash
   cd ~/robot_ws
   colcon build --symlink-install --packages-select hospital_robot_description
   source ~/robot_ws/install/setup.bash
   ```

4. **Check the XML parses before touching RViz** (clearer errors than RViz gives you):
   ```bash
   cd ~/robot_ws/src/hospital_robot_description/urdf
   xacro robot.urdf.xacro > /tmp/robot.urdf
   check_urdf /tmp/robot.urdf
   ```
   You should see a link tree with `base_footprint` at the root, then
   `base_link`, then your wheels/casters/lidar/column/imu underneath it.

5. **Launch it:**
   ```bash
   ros2 launch hospital_robot_description display.launch.py
   ```
   You should get RViz plus a small Joint State Publisher window with
   sliders.

6. **Go through the acceptance checklist from your original doc's Part 6.4** —
   robot visible (not just axes), facing +X, wheels touching the grid, wheel
   sliders spin the wheels about their own axle (not orbiting), column
   slider raises the camera with it, TF tree fully connected. This is where
   you'll actually confirm or correct the four "VERIFY" numbers above.

If something doesn't look right, your original document's **Troubleshooting**
table near the end covers every common symptom (floating robot, orbiting
wheel, empty RViz, mesh not showing, etc.) — it still applies unchanged.

---

## E. What's in this package

```
hospital_robot_description/
├── CMakeLists.txt
├── package.xml
├── urdf/
│   ├── robot.urdf.xacro       ← your numbers, already filled in
│   └── inertial_macros.xacro  ← placeholder inertia formulas, unchanged
├── launch/display.launch.py
├── rviz/display.rviz
└── meshes/                    ← put your STLs here (see Part C)
```

Once it's building and you're happy with the acceptance checklist, commit it
(`git init`, `.gitignore` for `build/ install/ log/`, `git add .`,
`git commit`) before you make any more changes — that way if an AI assistant
or you rewrite something and it breaks, `git diff` shows you exactly what
changed.
