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
| **D-pad Left/Right** | **− / + 50 RPM** while TEST MODE is enabled (applies immediately) |
| **D-pad Down** | **Disable TEST MODE** and **STOP** launcher |
| **Start** | **StopAll toggle** — latches an all-systems stop; press again to resume |

### Gamepad 2 – Co-Driver
| Control | Function |
|---|---|
| **Left Bumper (LB)** | **Feed once** (with **Intake Assist** if Intake is OFF) |
| **Right Bumper (RB)** | **Toggle Intake On/Off** |
| **Y / Triangle** | **Toggle AutoSpeed** (mirrors G1) |
| **Start** | **StopAll toggle** (same behavior as G1) |

**Startup defaults:**
`AutoSpeed = OFF`, `AutoAim = OFF`, `Intake = OFF` (edit in `config/TeleOpDriverDefaults.java`).

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
    │   └── VisionTuning.java                 ← AprilTag range scale calibration
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
- **Left stick movement** still provides full translation.  

### AutoSpeed
- When **enabled**, AutoSpeed calculates launcher RPM from AprilTag distance via `LauncherAutoSpeedController`.  
- When **disabled**, right trigger controls RPM directly.  
- **Defaults:**  
  - `InitialAutoDefaultSpeed = 2500 RPM` (used before first tag detection)  
  - AutoRPM interpolation between `(24 in → 1000 RPM)` and `(120 in → 4500 RPM)`  
  - Holds last valid RPM when tag not visible after first fix.  

### Manual Launcher Mode
- In manual (AutoSpeed = OFF), right trigger scales between `rpmBottom` and `rpmTop`.  
- If `rpmBottom > 0`, the launcher idles at that RPM even when trigger = 0.  
- Manual lock (X/Square) freezes current RPM until unlocked.  

### Intake, Feed, and Eject
- `DEFAULT_INTAKE_ENABLED` determines initial intake state.  
- Feeding automatically enables intake for `intakeAssistMs = 250 ms` if it was off.  
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
- **Recommended Resolution:** 640×480 MJPEG for smooth frame rate.

**Aim Controller Defaults**
```
kP = 0.02
kD = 0.003
twistClamp = ±0.6
deadband = 1.5°
```
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
While STOPPED, TeleOp ignores control outputs and keeps mechanisms at zero power.  
Press **Start** again to **RESUME** normal control.

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
