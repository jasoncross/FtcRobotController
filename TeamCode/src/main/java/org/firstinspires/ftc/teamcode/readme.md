# Indianola Robotics – FTC 2025 Robot Code · DECODE Season

## Project Overview
This repository contains the complete robot control code for the **FTC 2025–2026 DECODE** season.  
The robot uses a **mecanum drivetrain**, **dual flywheel launcher**, **feed motor**, **intake**, and **AprilTag-based targeting**.

All TeleOps and Autos share the same subsystem classes for consistency and easy maintenance.  
The codebase follows a modular architecture designed for both student development and future maintainers.

---

## Controller Bindings System

### Purpose
All controller input mappings are centralized in **`ControllerBindings.java`**.  
This allows any driver button or trigger assignments to be changed in one place  
without modifying the TeleOp or subsystem code.

### Location
```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/input/ControllerBindings.java
```

### Features
- Press, Hold, and Toggle detection (built-in debouncing)  
- Axis mapping for triggers (LT/RT)  
- Support for both Gamepad 1 and Gamepad 2  
- Optional rear-paddle (M1/M2) readers  
- Fully compatible with existing subsystems and TeleOp logic

---

## Controller Layout (Default Bindings)

| Control | Function |
|---|---|
| **Left Stick** | Drive (forward/back & strafe) |
| **Right Stick X** | Rotation (**disabled while AutoAim is ON**) |
| **Left Trigger** | Brake – reduces top speed |
| **Right Trigger** | Manual RPM control (**only** when **AutoSpeed = OFF**, **Lock = OFF**, **Test = OFF**) |
| **Left Bumper (LB)** | **Feed once** (with **Intake Assist** if Intake is OFF) |
| **Right Bumper (RB)** | **Toggle Intake On/Off** |
| **Right Stick Button (RS)** | **Toggle AutoAim** *(only ENABLES if a goal AprilTag is visible; auto-DISABLES if tag remains lost beyond grace window)* |
| **Y / Triangle** | **Toggle AutoSpeed** *(double-pulse on ENABLE, single-pulse on DISABLE)* |
| **X / Square** | **Toggle Manual RPM LOCK** *(only when AutoSpeed = OFF; holds current RPM)* |
| **B / Circle** | **Eject** *(temporary RPM = `EjectRPM`, feeds once with Intake Assist, then restores prior RPM)* |
| **D-pad Up** | **Enable RPM TEST MODE** |
| **D-pad Left/Right** | **− / + 50 RPM** while TEST MODE is enabled; when **AutoSpeed = OFF** **and Manual Lock = ON**, nudges manual RPM by `LauncherTuning.MANUAL_RPM_STEP` (default 50) |
| **D-pad Down** | **Disable TEST MODE** and **STOP** launcher |
| **Start** | **StopAll toggle** — latches an all-systems stop; press again to resume |

### Gamepad 2 – Co-Driver
| Control | Function |
|---|---|
| **Left Bumper (LB)** | **Feed once** (with **Intake Assist** if Intake is OFF) |
| **Right Bumper (RB)** | **Toggle Intake On/Off** |
| **Y / Triangle** | **Toggle AutoSpeed** (mirrors G1) |
| **D-pad Left** | **Select vision P480 profile** (640×480@30 performance stream) |
| **D-pad Right** | **Select vision P720 profile** (1280×720@20 sighting stream) |
| **D-pad Up** | **Enable Vision live view** (Driver Station preview on) |
| **D-pad Down** | **Disable Vision live view** (performance mode, preview off) |
| **Start** | **StopAll toggle** (same behavior as G1) |

**Startup defaults:**
`AutoSpeed = ON`, `AutoAim = OFF`, `Intake = ON` (edit in `config/TeleOpDriverDefaults.java`).

---

## Project Layout
```
TeamCode/
└── src/main/java/org/firstinspires/ftc/teamcode/
    Alliance.java                        ← Alliance enum for selecting RED/BLUE behaviors
    ├── assist/
    │   └── AutoAimSpeed.java                 ← Shared AutoAim + AutoSpeed helper
    ├── auto/
    │   ├── BaseAuto.java                     ← Shared Auto mode logic
    │   ├── Auto_Blue_Target.java             ← Blue depot auto (Tag 20 volley, hold position)
    │   ├── Auto_Blue_Human.java              ← Blue human-side auto (Tag 20 volley → drive upfield)
    │   ├── Auto_Red_Target.java              ← Red depot auto (Tag 24 volley, hold position)
    │   └── Auto_Red_Human.java               ← Red human-side auto (Tag 24 volley → drive upfield)
    ├── config/
    │   ├── AutoAimTuning.java                ← AutoAim overrides (twist, RPM seed)
    │   ├── AutoRpmConfig.java                ← Distance→RPM curve + smoothing
    │   ├── ControllerTuning.java             ← Trigger thresholds
    │   ├── DriveTuning.java                  ← Wheel geometry + IMU turn gains
    │   ├── FeedTuning.java                   ← Feed power, duration, cooldown
    │   ├── IntakeTuning.java                 ← Intake motor power
    │   ├── LauncherTuning.java               ← Flywheel clamps, PIDF, at-speed window
    │   ├── SharedRobotTuning.java            ← Cross-mode cadence, caps, IMU orientation
    │   ├── TeleOpDriverDefaults.java         ← Driver preferences & manual ranges
    │   ├── TeleOpEjectTuning.java            ← Eject RPM + timing
    │   ├── TeleOpRumbleTuning.java           ← Haptic envelopes
    │   └── VisionTuning.java                 ← AprilTag range scale + camera profile/intrinsics tunables
    ├── control/
    │   └── LauncherAutoSpeedController.java  ← Distance→RPM mapping + smoothing for AutoSpeed
    ├── drive/
    │   └── Drivebase.java                    ← Main driving logic; IMU orientation: Label UP, USB RIGHT
    ├── input/
    │   └── ControllerBindings.java           ← Centralized gamepad mapping/edge-detect helpers
    ├── subsystems/
    │   ├── Launcher.java                 ← Dual-flywheel subsystem (PIDF + AutoSpeed hooks)
    │   ├── Feed.java                     ← Feed motor timing + interlocks
    │   └── Intake.java                   ← Intake motor helper + assist timings
    ├── teleop/
    │   ├── TeleOpAllianceBase.java           ← Shared TeleOp logic (launcher modes, assists)
    │   ├── TeleOp_Blue.java                  ← Blue-side TeleOp wrapper (preselect + rumble cues)
    │   └── TeleOp_Red.java                   ← Red-side TeleOp wrapper (preselect + rumble cues)
    ├── utils/
    │   └── ObeliskSignal.java            ← LED/signal helpers for Obelisk status patterns
    └── vision/
        ├── VisionAprilTag.java           ← VisionPortal wrapper exposing Tag distance/pose
        └── TagAimController.java         ← PID twist controller for Tag-centered aiming
```


---

## TeleOp Behaviors & Tunables

For a complete, always-current list of adjustable parameters, see the
[TeamCode Tunable Directory](./TunableDirectory.md). It captures
where each value lives, which game modes it influences, and how overlapping
TeleOp/Auto tunables override one another. TeleOp driver preferences (startup
states, rumble envelopes, eject behavior, etc.) now live in
`config/` files, so you can retune match workflow without touching the
OpMode source.

For broader context on how the subsystems, StopAll latch, and rule constraints interconnect, review the
[Codex Context & Development Background](./CodexContextBackground.md) companion document.

### AutoAim
- **Enable:** Only when a goal AprilTag is visible.
- **Grace period:** If the tag is lost, AutoAim waits **`autoAimLossGraceMs = 4000` ms** before disabling.
  - If the tag reappears within that window → AutoAim continues automatically.
  - If not → AutoAim disables and provides a **single rumble pulse**.
- **Behavior:** While AutoAim (or grace) is active, **right stick rotation is ignored**.
  AutoAim continuously applies twist correction from `TagAimController` to hold target at 0°.
- **Translation is scaled** by `AutoAimTuning.AUTO_AIM_SPEED_SCALE` (default **0.25**) whenever AutoAim is ON; telemetry surfaces the active scale as `SpeedScale` to remind drivers how much throttle remains.

### AutoSpeed
- When **enabled**, AutoSpeed calculates launcher RPM from AprilTag distance via `LauncherAutoSpeedController`.
- When **disabled**, right trigger controls RPM directly.
- **Defaults:**
  - AutoRPM interpolation between **65.4 in → 4550 RPM** and **114 in → 5000 RPM** (`AutoRpmConfig` anchors)
  - **Default hold** while no tag is visible = **4450 RPM** (`AutoRpmConfig.DEFAULT_NO_TAG_RPM`)
  - Holds the **last vision-derived RPM** once at least one tag fix has occurred.

### Manual Launcher Mode
- In manual (AutoSpeed = OFF), right trigger scales between `rpmBottom` and `rpmTop`.
- If `rpmBottom > 0`, the launcher idles at that RPM even when trigger = 0.
- Manual lock (X/Square) freezes current RPM until unlocked.
- D-pad left/right apply ±`LauncherTuning.MANUAL_RPM_STEP` adjustments for quick fine-tuning **only while Manual Lock is engaged** (keeps lock and AutoSpeed off).

### Intake, Feed, and Eject
- `DEFAULT_INTAKE_ENABLED` determines initial intake state; `safeInit()` keeps the motor idle during INIT before defaults apply.
- Feeding automatically enables intake for `intakeAssistMs = FeedTuning.INTAKE_ASSIST_MS` (default `250 ms`) if it was off.
- Feed motor holds position with BRAKE zero-power behavior; idle counter-rotation (`FeedTuning.IDLE_HOLD_POWER`, default `-0.5`) only enables after START.
- **Eject (B/Circle):** runs launcher at `TeleOpEjectTuning.RPM` (default `600 RPM`) for `TeleOpEjectTuning.TIME_MS` (default `1000 ms`), feeds once, then restores the previous RPM.

### Haptics
- **Double pulse:** feature enabled.  
- **Single pulse:** feature disabled or AutoAim grace expired.  
- Aim rumble scales by heading error (only active when AutoAim = OFF).  

---

## Vision (AprilTags)

- **Camera:** “Webcam 1” via VisionPortal and AprilTagProcessor.
- **Alliance goals:** Blue = Tag 20  |  Red = Tag 24
- **Distance units:** inches = meters × 39.37
- **Range scaling:** `vision.setRangeScale(trueMeters / measuredMeters)` adjusts calibration.
- **Vision profiles** (`config/VisionTuning.java → P480_* / P720_*` constants via `VisionTuning.forMode(...)`):
  - **P480 (Performance):** 640×480 @ 30 FPS, AprilTag decimation = `2.8`, processes every frame, minimum decision margin = `25`, manual exposure = `10 ms`, gain = `95`, white balance lock = `true`, Brown–Conrady intrinsics/distortion for Logitech C270 (fx = fy = 690, cx = 320, cy = 240, k1 = −0.27, k2 = 0.09, p1 = 0.0008, p2 = −0.0006).
  - **P720 (Sighting):** 1280×720 @ 20 FPS, AprilTag decimation = `2.2`, processes every other frame, minimum decision margin = `38`, manual exposure = `15 ms`, gain = `110`, white balance lock = `true`, calibrated intrinsics/distortion (fx = 1380, fy = 1035, cx = 640, cy = 360, k1 = −0.23, k2 = 0.06, p1 = 0.0005, p2 = −0.0005).
- **Startup defaults:** Profile = **P480**, live view **OFF** (no Driver Station preview).
- **Streaming toggle:** Gamepad 2 D-pad up/down calls `vision.toggleLiveView(...)` (prefers MJPEG preview when enabled).
- **Telemetry bundle (≈10 Hz):**
  - `Vision: Profile=<P480|P720> LiveView=<ON|OFF> Res=<WxH>@<FPS> Decim=<x.x> ProcN=<n> MinM=<m>`
  - `Perf: FPS=<measured> LatMs=<latest>`
- **Driver feedback:** Telemetry raises a one-time warning if the webcam does not accept manual exposure/gain/white-balance commands.

**Aim Controller Defaults**
```
kP = 0.02
kD = 0.003
twistClamp = ±0.6
deadband = 1.5°
```
---
## Autonomous Routines (2025-10-31 Refresh)

All autonomous modes extend `BaseAuto`, which now surfaces a shared telemetry bundle every loop:

- **Alliance** (BLUE/RED)
- **Auto** (OpMode display name)
- **Start Pose** (human-readable staging reminder)
- **Obelisk** (`ObeliskSignal.getDisplay()` live latch)
- **AprilTag Lock** (`LOCKED`/`SEARCHING`)
- **Phase** (current step description from the active helper)

While those lines remain visible, the helper methods continue to enforce **no-shot-without-lock** and **±50 RPM at-speed** gating before the feed motor ever cycles. `SharedRobotTuning.SHOT_BETWEEN_MS` (default **3000 ms**) spaces each ball in the three-round volleys, and `stopAll()` plus a `"Auto complete – DS will queue TeleOp."` telemetry banner finish every routine. `VisionAprilTag` keeps the Obelisk AprilTag observer running in the background during all phases so the latched motif carries into TeleOp.

### 🔵 Auto_Blue_Target – Depot launch line, facing EAST
1. **Drive forward 36"** to establish the standoff range.
2. **Scan counter-clockwise** until AprilTag 20 centers within ±1°.
3. **Spin the launcher** with AutoSpeed until the wheels are within ±50 RPM of target.
4. **Fire three balls** with ~3 s spacing enforced by `SharedRobotTuning.SHOT_BETWEEN_MS`.
5. **Hold position** for the remainder of the autonomous period.

### 🔴 Auto_Red_Target – Depot launch line, facing WEST
1. **Drive forward 36"** to the calibrated firing spot.
2. **Scan clockwise** for AprilTag 24 and settle within ±1°.
3. **Spin the launcher** to target RPM (±50 RPM tolerance).
4. **Fire three balls** with ~3 s spacing.
5. **Remain parked** to leave the lane clear for the partner bot.

### 🔵 Auto_Blue_Human – West of south firing triangle, facing NORTH
1. **Record heading and bump forward 2"** to clear the wall.
2. **Scan counter-clockwise** for AprilTag 20 until centered within ±1°.
3. **Spin the launcher** to target RPM with the shared tolerance window.
4. **Fire three balls** with the enforced 3 s cadence.
5. **Return to the original heading**, honoring the recorded IMU value.
6. **Drive forward 24"** toward the classifier lane.

### 🔴 Auto_Red_Human – East of south firing triangle, facing NORTH
1. **Record heading and bump forward 2"** to clear the wall.
2. **Scan clockwise** for AprilTag 24 until centered within ±1°.
3. **Spin the launcher** into the ±50 RPM window.
4. **Fire three balls** separated by ~3 s.
5. **Return to the starting heading** using the shared IMU helper.
6. **Drive forward 24"** upfield toward the classifier.

> **Common Safeguards** – All modes call `updateStatus(...)` while scanning, spinning, and firing so drivers can verify the tag lock, RPM, and Obelisk state live. Feeding never occurs unless both lock and at-speed checks succeed, aimSpinUntilReady() now seeds launcher RPM through AutoSpeed (honoring `AutoRpmConfig.DEFAULT_NO_TAG_RPM`) before the first tag lock, and the launcher target resets to the configured hold RPM if vision drops. Startup states now mirror TeleOp: the intake enables only after START, feed idle hold engages once the match begins, and stopAll() releases the counter-rotation just like the TeleOp latch.

---
## Obelisk AprilTag Signal (DECODE 2025–26)

### Overview
The on-field **obelisk** displays one of three AprilTags that determine the **optimal ball order** for bonus points:

| Tag ID | Pattern | Meaning |
|:------:|:--------|:--------|
| **21** | GPP | Green → Purple → Purple |
| **22** | PGP | Purple → Green → Purple |
| **23** | PPG | Purple → Purple → Green |

### Behavior
- The robot continuously scans for these tags via `VisionAprilTag.observeObelisk()`.  
- When detected, the shared class `ObeliskSignal` latches the pattern (`GPP`, `PGP`, or `PPG`) in memory.  
- This value persists between Auto and TeleOp modes so both can access the same detected order.  
- **Telemetry:** The first line on the Driver Station always shows the current obelisk result,  
  e.g. `Obelisk: PGP (10 s ago)`.

### Implementation Details
| File | Purpose |
|------|----------|
| [`vision/VisionAprilTag.java`](./vision/VisionAprilTag.java) | Detects AprilTags 21/22/23 and updates shared state via `observeObelisk()`. |
| [`utils/ObeliskSignal.java`](./utils/ObeliskSignal.java) | In-memory latch that stores and displays the detected obelisk order. |
| [`auto/BaseAuto.java`](./auto/BaseAuto.java) | Observes the obelisk during the **prestart** loop, allowing the robot to lock in the signal before the match begins. |
| [`teleop/TeleOpAllianceBase.java`](./teleop/TeleOpAllianceBase.java) | Calls `vision.observeObelisk()` every loop and displays the latched order on the **first telemetry line**. |

---

## Tunable Directory

The detailed directory of tunable values lives in
[TeamCode Tunable Directory](./TunableDirectory.md). Review that
document for authoritative defaults, tuning guidance, and notes on which class
or game mode owns each parameter before making adjustments.

---
## StopAll & Auto-Stop Timer (NEW)

### What is StopAll?
`StopAll` immediately commands **drive, launcher, feed, and intake** to stop and **latches** a STOPPED state.
While STOPPED, TeleOp ignores control outputs, keeps mechanisms at zero power, and temporarily disables the feed motor's idle
hold so the motor rests at 0.
Press **Start** again to **RESUME** normal control, which restores the idle hold automatically.

- Engaged manually any time by pressing **Start** (G1 or G2).  
- Also executed automatically by the **Auto-Stop timer** when enabled and the countdown reaches zero.  
- `stopAll()` is also called in `OpMode.stop()` for safety.

### Auto-Stop Timer
- **Parameters:**
  - `autoStopTimerEnabled` (default **false**)
  - `autoStopTimerTimeSec` (default **119**)
- **Behavior:**
  - When enabled, the timer **starts at TeleOp INIT**.
  - A **top-line telemetry countdown** is shown only while enabled.
  - At 0, the timer **engages StopAll** (latches STOPPED). Press **Start** to resume if needed.

---

## Development Context

- **Architecture:** Mecanum drive + IMU heading control.  
- **Launcher:** Dual goBILDA 5202 6000 RPM motors, closed-loop PID.  
- **Vision:** AprilTag ID 20/24 goal targeting.  
- **Telemetry:** Drive, launcher RPM, AutoSpeed state, AutoAim status, tag distance + heading.  
- **File header standard:** `FILE / LOCATION / PURPOSE / NOTES / METHODS`.  
- **Rule Reference:** FTC 2025–2026 Competition Manual + Team Updates.  

---

## Revision History
- **2025‑10‑31** – Added Logitech C270 vision profiles (P480 performance + P720 sighting) with per-profile decimation, gating, camera controls, and Brown–Conrady calibration, defaulted TeleOp to P480 with live view off, exposed Gamepad 2 D-pad bindings to swap profiles or toggle the live preview, condensed telemetry into `Vision` + `Perf` status lines, refactored `VisionTuning` into P480/P720 constant blocks with a `forMode(...)` helper while preserving legacy fields, retuned AutoRPM anchors to 65.4 in → 4550 RPM and 114 in → 5000 RPM with a 4450 RPM default hold when tags drop, ensured both TeleOp and Auto seed launcher RPM exclusively through AutoSpeed so BaseAuto now idles at the AutoRpmConfig default before first tag lock, refined AutoSpeed so that default RPM only seeds the first lock before holding the last vision-computed RPM, added subsystem `safeInit()` gating so all motors stay idle through INIT, defaulted TeleOp AutoSpeed + intake to ON, raised the feed idle counter-rotation to −0.5 by default, ensured StopAll disables the feed idle hold until Start resumes TeleOp control, **and refreshed all four autonomous routines** (Blue/Red Target + Human) to follow the latest match playbook: 36" depot standoffs, 2" wall-clear bumps on human starts, ±1° tag lock + ±50 RPM gating before every shot, 3 s cadence spacing, 24" human-lane pushes, enhanced telemetry (Alliance/Auto/Start Pose/Obelisk/Tag Lock/Phase), persistent Obelisk observation, and a shared "Auto complete – DS will queue TeleOp." banner with `stopAll()` catch-all shutdown, and BaseAuto start/stop parity with TeleOp (intake auto-enables at START and feed idle hold releases inside stopAll()).
- **2025‑10‑30** – Added AutoAim translation speed scaling + telemetry, manual RPM D-pad nudges gated behind Manual Lock, feed motor brake guard, VisionPortal live stream, and moved `INTAKE_ASSIST_MS` into `FeedTuning`.
- **2025‑10‑26** – Added revision history to the readme.
- **2025‑10‑25** – All tuning parameters moved into separate config files; major commenting overhaul.
- **2025‑10‑23** – Controller rumble feedback added; Intake assist logic implemented; eject function implemented; etc.
- **2025‑10‑22** – Initial DECODE TeleOp base with AutoSpeed & AutoAim integration.

---

## Credits
Indianola Robotics – FTC Team 2025  
Mentor Support: *Jason Cross*  
Some portions of this code and documentation were created or refined with the assistance of OpenAI's ChatGPT Codex under mentor supervision. All final design, testing, and implementation decisions were made by Indianola Robotics Team.

Built on the official **FIRST Tech Challenge SDK**  

---

© Indianola Robotics · DECODE Season (2025–2026)
