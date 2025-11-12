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
| **Left Stick Button (LS)** | Toggle **Reverse Drive** (treats rear as front; double rumble when enabled, single when disabled) |
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
    │   ├── BaseAuto.java                     ← Shared Auto mode logic + AutoSequence builder
    │   ├── Auto_Blue_30.java                 ← Blue alliance safety auto (drive 30" and stop)
    │   ├── Auto_Blue_Human.java              ← Blue human-side auto (Tag 20 long-run volley → retreat)
    │   ├── Auto_Blue_Human_LongShot.java     ← Blue human-side launch-line volley → drive upfield
    │   ├── Auto_Blue_Target.java             ← Blue depot auto (Tag 20 volley, hold position)
    │   ├── Auto_Red_30.java                  ← Red alliance safety auto (drive 30" and stop)
    │   ├── Auto_Red_Human.java               ← Red human-side auto (Tag 24 long-run volley → retreat)
    │   ├── Auto_Red_Human_LongShot.java      ← Red human-side launch-line volley → drive upfield
    │   ├── Auto_Red_Target.java              ← Red depot auto (Tag 24 volley, hold position)
    │   └── AutoSequenceGuide.md              ← Reference + examples for the AutoSequence builder
    ├── config/
    │   ├── AutoAimTuning.java                ← AutoAim overrides (twist, RPM seed)
    │   ├── AutoRpmConfig.java                ← Distance→RPM curve + smoothing
    │   ├── ControllerTuning.java             ← Trigger thresholds
    │   ├── DriveTuning.java                  ← Wheel geometry + IMU turn gains
    │   ├── FeedTuning.java                   ← Feed power, duration, cooldown
    │   ├── FeedStopConfig.java               ← Feed gate servo scaling + timing
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
    │   ├── TeleOp_Red.java                   ← Red-side TeleOp wrapper (preselect + rumble cues)
    │   └── TeleOp_Test_CameraStream.java     ← Diagnostics TeleOp for streaming + AprilTag checks
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
- **Driver toggles:** Gamepad Y buttons queue AutoSpeed enable/disable requests so the TeleOp loop finishes scanning
  controls before seeding RPM or emitting rumble pulses—drive/aim inputs stay live while the launcher mode flips.

### Manual Launcher Mode
- In manual (AutoSpeed = OFF), right trigger scales between `rpmBottom` and `rpmTop`.
- If `rpmBottom > 0`, the launcher idles at that RPM even when trigger = 0.
- Manual lock (X/Square) freezes current RPM until unlocked.
- D-pad left/right apply ±`LauncherTuning.MANUAL_RPM_STEP` adjustments for quick fine-tuning **only while Manual Lock is engaged** (keeps lock and AutoSpeed off).

### Intake, Feed, and Eject
- `DEFAULT_INTAKE_ENABLED` determines initial intake state; `safeInit()` keeps the motor idle during INIT before defaults apply.
- Feeding automatically enables intake for `intakeAssistMs = FeedTuning.INTAKE_ASSIST_MS` (default `250 ms`) if it was off.
- Feed motor holds position with BRAKE zero-power behavior; idle counter-rotation (`FeedTuning.IDLE_HOLD_POWER`, default `-0.5`) only enables after START.
- Feed/Eject commands now ride the Feed subsystem's asynchronous cycle, so the TeleOp loop keeps processing drive/aim inputs while the feed motor pulses and the intake assist timer counts down in the background.
- FeedStop servo (`config/FeedStopConfig.java`) now homes in two guarded phases: it first steps open in the release direction to `SAFE_PRESET_OPEN_DEG` (capped by `MAX_HOME_TRAVEL_DEG`) without ever commanding below 0°, then seats against the BLOCK stop, dwells for `HOME_DWELL_MS`, and backs off by `HOME_BACKOFF_DEG` before parking. Every degree request is clamped inside `SOFT_CCW_LIMIT_DEG` (0°) and `SOFT_CW_LIMIT_DEG` (170°), so no runtime command can crash the linkage. After homing it rests at `HOLD_ANGLE_DEG` (~30°) to block the path, swings to `RELEASE_ANGLE_DEG` (~110°) when feeding, and defaults to the servo’s full 300° span (no `scaleRange`). Teams that enable `USE_AUTO_SCALE` let the subsystem compute the narrowest safe window (with `SAFETY_MARGIN_DEG` headroom) and telemetry now surfaces the mode, limits, scale range (or “scale=none”), direction sign, and any clamp/abort warnings. StopAll/stop() always return the gate to the homed 0° position before disabling.
- **Eject (B/Circle):** runs launcher at `TeleOpEjectTuning.RPM` (default `600 RPM`) for `TeleOpEjectTuning.TIME_MS` (default `1000 ms`), feeds once, then restores the previous RPM.
  The spool → feed → hold sequence is asynchronous, so drivers can keep steering (or cancel with StopAll) while the timer winds down.

### Haptics
- **Double pulse:** feature enabled.
- **Single pulse:** feature disabled or AutoAim grace expired.
- Aim rumble scales by heading error (only active when AutoAim = OFF).

### Reverse Drive Mode
- **Toggle:** Gamepad 1 **Left Stick Button (LS)**.
- **Behavior:** Inverts forward/back and strafe commands so the rear behaves as the front while leaving twist control unchanged.
- **Feedback:** Emits a **double rumble** when enabled and a **single rumble** when disabled to match other mode toggles.

### Camera Stream Diagnostics Mode
- **OpMode:** `X - Test - Camera Stream` (Test group) – launches a minimal loop that keeps only drivetrain drive/strafe/twist inputs, live AprilTag processing, and the webcam stream active so pits can verify focus and alignment without spinning up other subsystems.
- **Streaming:** Automatically enables the Driver Station preview on init; telemetry surfaces the active profile, FPS, and latency so crews can gauge pipeline health at a glance.
- **Resolution swaps:** Gamepad 1 D-pad **left** selects the tuned 640×480 performance profile; D-pad **right** selects the 1280×720 sighting profile. Swaps rebuild the VisionPortal in a background thread while maintaining the live stream.
- **Telemetry focus:** Displays the nearest detected tag ID, scaled range in inches, and bearing so camera aim tweaks can be confirmed immediately. All other TeleOp automations (feed, launcher, rumble, StopAll) remain idle to minimize Control Hub load during testing.

---

## Vision (AprilTags)

- **Camera:** “Webcam 1” via VisionPortal and AprilTagProcessor.
- **Alliance goals:** Blue = Tag 20  |  Red = Tag 24
- **Distance units:** inches = meters × 39.37
- **Range scaling:** `vision.setRangeScale(trueMeters / measuredMeters)` adjusts calibration.
- **Shared initialization:** TeleOp and BaseAuto both call `vision.setRangeScale(VisionTuning.RANGE_SCALE)` so distance math and AutoSpeed RPM seeds stay consistent between match phases.
- **Vision profiles** (`config/VisionTuning.java → P480_* / P720_*` constants via `VisionTuning.forMode(...)`):
  - **P480 (Performance):** 640×480 @ 30 FPS, AprilTag decimation = `2.8`, processes every frame, minimum decision margin = `25`, manual exposure = `10 ms`, gain = `95`, white balance lock = `true`, Brown–Conrady intrinsics/distortion for Logitech C270 (fx = fy = 690, cx = 320, cy = 240, k1 = −0.27, k2 = 0.09, p1 = 0.0008, p2 = −0.0006).
  - **P720 (Sighting):** 1280×720 @ 20 FPS, AprilTag decimation = `2.2`, processes every other frame, minimum decision margin = `38`, manual exposure = `15 ms`, gain = `110`, white balance lock = `true`, calibrated intrinsics/distortion (fx = 1380, fy = 1035, cx = 640, cy = 360, k1 = −0.23, k2 = 0.06, p1 = 0.0005, p2 = −0.0005).
  - **Startup defaults:** Profile = **P480**, live view **OFF** (no Driver Station preview).
  - **Runtime swaps:** TeleOp now queues profile changes on a background executor so the VisionPortal rebuild does not pause drive control when drivers tap D-pad left/right.
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
## Autonomous Routines (2025-11-02 Warm-Up Refresh)

All autonomous modes extend `BaseAuto`, which now surfaces a shared telemetry bundle every loop:

- **Alliance** (BLUE/RED)
- **Auto** (OpMode display name)
- **Start Pose** (human-readable staging reminder)
- **Obelisk** (`ObeliskSignal.getDisplay()` live latch)
- **AprilTag Lock** (`LOCKED`/`SEARCHING`)
- **Phase** (current step description from the active helper)

While those lines remain visible, the helper methods continue to enforce **no-shot-without-lock** and **±50 RPM at-speed** gating before the feed motor ever cycles. Each firing step now supplies its own between-shot delay (the stock autos use **3000 ms**) and `spinToAutoRpmDefault(...)` keeps the launcher warm while the robot drives or scans. `stopAll()` plus a `"Auto complete – DS will queue TeleOp."` telemetry banner finish every routine. `VisionAprilTag` keeps the Obelisk AprilTag observer running in the background during all phases so the latched motif carries into TeleOp.

Refer to the [AutoSequence Builder Guide](./auto/AutoSequenceGuide.md) for the fluent API’s method reference and examples.

### 🔵 Auto_Blue_Target – Depot launch line, facing EAST
1. **Drive forward 36"** to establish the standoff range.
2. **Pre-spin the launcher** to the AutoSpeed default while holding heading.
3. **Scan counter-clockwise** until AprilTag 20 centers within ±1°.
4. **Ready the launcher** with AutoSpeed until the wheels hold within ±50 RPM for the shared settle window.
5. **Fire three artifacts** with ~3 s spacing.
6. **Hold position** for the remainder of the autonomous period.

### 🔴 Auto_Red_Target – Depot launch line, facing WEST
1. **Drive forward 36"** to the calibrated firing spot.
2. **Pre-spin the launcher** to the AutoSpeed default while holding heading.
3. **Scan clockwise** for AprilTag 24 and settle within ±1°.
4. **Ready the launcher** to target RPM (±50 RPM tolerance with settle).
5. **Fire three artifacts** with ~3 s spacing.
6. **Remain parked** to leave the lane clear for the partner bot.

### 🔵 Auto_Blue_Human – West of south firing triangle, facing NORTH
1. **Record heading and bump forward 2"** to clear the wall.
2. **Pre-spin the launcher** to the AutoSpeed default while holding heading.
3. **Scan counter-clockwise** for AprilTag 20 until centered within ±1°.
4. **Ready the launcher** to target RPM with the shared tolerance + settle window.
5. **Fire three artifacts** with the 3 s cadence.
6. **Return to the original heading**, honoring the recorded IMU value.
7. **Drive forward 24"** toward the classifier lane.

### 🔴 Auto_Red_Human – East of south firing triangle, facing NORTH
1. **Record heading and bump forward 2"** to clear the wall.
2. **Pre-spin the launcher** to the AutoSpeed default while holding heading.
3. **Scan clockwise** for AprilTag 24 until centered within ±1°.
4. **Ready the launcher** into the ±50 RPM window with the shared settle timer.
5. **Fire three artifacts** separated by ~3 s.
6. **Return to the starting heading** using the shared IMU helper.
7. **Drive forward 24"** upfield toward the classifier.

> **Common Safeguards** – All modes call `updateStatus(...)` while scanning, spinning, and firing so drivers can verify the tag lock, RPM, and Obelisk state live. Feeding never occurs unless both lock and at-speed checks succeed, `readyLauncherUntilReady()` now shares the TeleOp AutoSpeed curve while seeding from `SharedRobotTuning.INITIAL_AUTO_DEFAULT_SPEED` and waiting out `SharedRobotTuning.RPM_READY_SETTLE_MS`, and the launcher target resets to the configured hold RPM if vision drops. Startup states now mirror TeleOp: the intake enables only after START, feed idle hold engages once the match begins, and stopAll() releases the counter-rotation just like the TeleOp latch.

---
## Obelisk AprilTag Signal (DECODE 2025–26)

### Overview
The on-field **obelisk** displays one of three AprilTags that determine the **optimal artifact order** for bonus points:

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
While STOPPED, TeleOp ignores control outputs, reasserts **BRAKE zero-power behavior on every motor**, keeps mechanisms at zero power,
and temporarily disables the feed motor's idle hold so the motor rests at 0.
The launcher automatically returns to FLOAT the next time RPM is commanded so normal spin-up behavior resumes after releasing the latch.
Press **Start** again to **RESUME** normal control, which restores the idle hold automatically and re-applies the intake's prior ON/OFF state so drivers pick up exactly where they left off.

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
- **2025-11-13** – Refreshed all autonomous header comments to document the new long-run, launch-line long-shot, and 30" safety routes (noting five-shot cadence, retreat/advance plans, and vision swaps) and added the new auto classes to the Project Layout tree for quick discovery.
- **2025-11-12** – Captured the live intake state before StopAll engages so resuming with Start restores whichever intake mode was active, eliminating the need to re-toggle the motor after manual or timer-triggered stops; documented the behavior in the StopAll section for drive team clarity.
- **2025-11-11** – Added the "X - Test - Camera Stream" diagnostic TeleOp that boots with live streaming enabled, limits control to drivetrain drive/strafe/twist plus AprilTag telemetry, and maps Gamepad 1 D-pad left/right to swap between the tuned 480p performance and 720p sighting profiles; documented the workflow and updated the project layout accordingly.
- **2025-11-10** – Added a TeleOp Reverse Drive mode toggled by the Gamepad 1 left stick button, inverting forward/strafe vectors while leaving twist intact, hooked the toggle into the shared rumble patterns (double on enable, single on disable), surfaced the mode state in telemetry, and updated the controller layout + Reverse Drive documentation for drivers.
- **2025-11-07** – Made TeleOp feed/eject routines asynchronous so driver inputs stay live during shots, added intake-assist timers tied to the new Feed state machine, updated BaseAuto to use the shared gating, refreshed docs to note the non-blocking behavior, reworked toggle rumble pulses so double-blip feedback no longer sleeps the TeleOp loop, moved TeleOp vision profile swaps onto a background executor so switching between P480/P720 no longer stalls the drive loop, queued AutoSpeed enable/disable requests so RPM seeding + rumble feedback happen after the control scan without pausing drive input, reworked the FeedStop to home at INIT, auto-scale the servo window for separate hold/release degree targets, ensure StopAll parks at the homed zero, and retire obsolete tunables with updated telemetry/docs, defaulted FeedStop to full-span servo travel with an optional auto-scale toggle, refreshed telemetry strings, cleaned up the docs/tunable listings, and added a two-phase guarded homing routine with soft-limit clamps, safe-open travel caps, auto-scale telemetry, and StopAll/stop-to-home safeguards documented for pit crews.
- **2025-11-06** – Integrated a FeedStop servo gate across Feed/TeleOp/BaseAuto, added `config/FeedStopConfig.java` tunables (scale, block/release, hold, lead), refreshed telemetry + StopAll handling so the gate re-latches cleanly, and updated docs/Tunable Directory to explain the new feed blocker behavior.
- **2025-11-05** – Aligned Autonomous range scaling with TeleOp by applying `VisionTuning.RANGE_SCALE` during BaseAuto init, added an `AutoSequence.visionMode(...)` builder step for runtime AprilTag profile swaps, updated both human-side autos to begin in the 720p sighting profile, and refreshed docs/Tunable Directory to describe the shared calibration helper.
- **2025-11-04** – Corrected the Autonomous `move(...)` forward vector so positive distances now drive upfield like TeleOp, added inline telemetry logging for raw/applied vectors to confirm heading math, documented the fix inside `Drivebase.java`, and updated `stopAll()` in TeleOp + Auto to reapply BRAKE mode on every drivetrain/subsystem motor (with the launcher restoring FLOAT on the next command) so endgame holds resist pushes from alliance partners.
- **2025-11-03** – Elevated AutoSequence telemetry labels so each phase now prints as the first line with a spacer before the shared status bundle, making the active step obvious while additional data (RPM, range, etc.) continues to append underneath. Later in the day we renamed the launcher prep step to `readyToLaunch(...)`, added a shared RPM settle timer (`SharedRobotTuning.RPM_READY_SETTLE_MS`), unified Auto launcher spin-up with the TeleOp AutoSpeed curve, refreshed telemetry (distance/target/actual/tolerance/remaining time) during launcher prep, and updated docs + autos to use `spinToAutoRpmDefault(...)`/`readyToLaunch(...)`.
- **2025‑11-02** – **Refreshed all four autonomous routines** (Blue/Red Target + Human) to follow the latest match playbook: 36" depot standoffs, 2" wall-clear bumps on human starts, ±1° tag lock + ±50 RPM gating before every shot, 3 s cadence spacing, 24" human-lane pushes, enhanced telemetry (Alliance/Auto/Start Pose/Obelisk/Tag Lock/Phase), persistent Obelisk observation, and a shared "Auto complete – DS will queue TeleOp." banner with `stopAll()` catch-all shutdown, BaseAuto start/stop parity with TeleOp (intake auto-enables at START and feed idle hold releases inside stopAll()), **plus a new `AutoSequence` builder that chains move/rotate/aim/fire steps with adjustable speed caps and optional tag-lock gating—rewriting all four autos with the fluent API resolved the backwards drive regression by standardizing drive power limits and makes future route tweaks a single-line edit. Documented the sequencing workflow in `auto/AutoSequenceGuide.md`, linked it from the README, and captured the expanded builder methods for future route authors.** Extended the builder with `spinToAutoRpm(...)`, updated all autos to pre-spin the launcher before tag scans, parameterized the volley cadence per sequence (removing the shared tunable), reaffirmed that AutoSpeed stays enabled during feeds, and migrated the detailed builder breakdown out of this README in favor of the dedicated guide. Added AutoSpeed hold protection inside `BaseAuto.fireN(...)` so the launcher never sags between artifacts and updated docs/terminology to call the scoring pieces artifacts consistently.
- **2025‑10-31** – Added Logitech C270 vision profiles (P480 performance + P720 sighting) with per-profile decimation, gating, camera controls, and Brown–Conrady calibration, defaulted TeleOp to P480 with live view off, exposed Gamepad 2 D-pad bindings to swap profiles or toggle the live preview, condensed telemetry into `Vision` + `Perf` status lines, refactored `VisionTuning` into P480/P720 constant blocks with a `forMode(...)` helper while preserving legacy fields, retuned AutoRPM anchors to 65.4 in → 4550 RPM and 114 in → 5000 RPM with a 4450 RPM default hold when tags drop, ensured both TeleOp and Auto seed launcher RPM exclusively through AutoSpeed so BaseAuto now idles at the AutoRpmConfig default before first tag lock, refined AutoSpeed so that default RPM only seeds the first lock before holding the last vision-computed RPM, added subsystem `safeInit()` gating so all motors stay idle through INIT, defaulted TeleOp AutoSpeed + intake to ON, raised the feed idle counter-rotation to −0.5 by default, ensured StopAll disables the feed idle hold until Start resumes TeleOp control.
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
