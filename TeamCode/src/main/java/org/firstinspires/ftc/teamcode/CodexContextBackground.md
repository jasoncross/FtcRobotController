# Indianola Robotics – FTC 2025 *DECODE* Season  
## Codex Context & Development Background

---

## 🧭 Project Overview

This `TeamCode` module contains the custom robot control stack for the **Indianola Robotics Octobots** in the **2025–2026 FTC Season: DECODE™ presented by RTX**. All code builds on the public FTC SDK that lives in the sibling `FtcRobotController` module.

Match strategy centers on consistent scoring of **ARTIFACTS** into **GOALS**, rapid interaction with **OBELISKS**, and building **PATTERNS** under the expansion and safety rules highlighted below. The robot architecture implemented in this branch provides:

- **Field-centric mecanum drivetrain** powered by [`drive/Drivebase.java`](./drive/Drivebase.java).
- **Dual flywheel launcher** with a distance-aware AutoSpeed curve managed by [`control/LauncherAutoSpeedController.java`](./control/LauncherAutoSpeedController.java).
- **Synchronized feed and intake subsystems** ([`subsystems/Feed.java`](./subsystems/Feed.java), [`subsystems/Intake.java`](./subsystems/Intake.java)).
- **AprilTag-based targeting** via [`vision/VisionAprilTag.java`](./vision/VisionAprilTag.java) and [`assist/AutoAimSpeed.java`](./assist/AutoAimSpeed.java).
- **Shared TeleOp and Auto frameworks** that reuse the same subsystems, tunables, and stop safeguards ([`teleop/TeleOpAllianceBase.java`](./teleop/TeleOpAllianceBase.java), [`auto/BaseAuto.java`](./auto/BaseAuto.java)).
- **StopAll latch with optional auto-stop timer** orchestrated inside [`teleop/TeleOpAllianceBase.java`](./teleop/TeleOpAllianceBase.java) for end-of-match compliance.

All OpModes run on centralized configuration tables ([`config/*.java`](./config)) so drivetrain geometry, launcher tuning, rumble envelopes, and driver defaults remain aligned between TeleOp and Autonomous. Reference the [TeamCode Tunable Directory](./TunableDirectory.md) for the authoritative list.

---

## ⚙️ Design Philosophy

Guided by the [TeleOp and subsystem headers](./teleop/TeleOpAllianceBase.java), development focused on:

1. **Reliability in both match phases** – encoder-driven translation, IMU-locked turns, and shared AutoAim/AutoSpeed loops keep TeleOp and Auto behavior identical.
2. **Subsystem modularity** – each mechanism exposes a clear API (`Drivebase.drive(...)`, `Launcher.setTargetRpm(...)`, `Feed.fireOnce(...)`) so TeleOp and Autonomous can evolve without forking logic.
3. **Driver usability** – the [`input/ControllerBindings`](./input/ControllerBindings.java) layer debounces buttons, tracks toggles, and feeds haptic cues through [`utils/RumbleNotifier`](./utils/RumbleNotifier.java) with rumble envelopes stored in [`config/TeleOpRumbleTuning`](./config/TeleOpRumbleTuning.java).
4. **Safety** – the StopAll latch zeros drivetrain, launcher, feed, and intake outputs instantly, and can trigger automatically through the timer configured in [`config/TeleOpDriverDefaults`](./config/TeleOpDriverDefaults.java).
5. **Transparency for students** – every Java class begins with a header detailing its purpose, tunables, and call structure, encouraging rapid onboarding for new coders.

---

## 📘 Rule Background (Inline Highlights)

Key DECODE rules that shaped the system are reinforced throughout `TeamCode` comments:

> **R105 (Team Update 00)** – Robots may expand horizontally after start but must remain within 18"×18" footprint; vertical expansion capped at 38". Violations risk penalties.
>
> **G414 (Competition Manual TU06)** – Expansion is legal only inside the R105 limits; breaching limits allows disablement.
>
> **G501 (Team Update 00)** – Up to eight DC motors and ten servos per robot using FTC-approved hardware.
>
> **R601 (Team Update 00)** – Robots must run approved 12 V battery packs routed through the main power switch.
>
> **Section 9.10 (Competition Manual TU06)** – AprilTags on field structures are the official localization references; vision-assisted alignment is legal and encouraged.

These constraints drive the emphasis on IMU-stable turning, safe power distribution, and accurate AprilTag alignment.

---

## 🧩 Subsystem & File Context

### 🧭 Drivebase ([`drive/Drivebase.java`](./drive/Drivebase.java))
- **Role:** Shared mecanum drivetrain abstraction for TeleOp (robot-centric drive) and Auto (encoder/IMU helpers).
- **Highlights:**
  - Configurable wheel geometry and strafing compensation from [`config/DriveTuning`](./config/DriveTuning.java).
  - Dual constructors support blocking motion helpers in Auto and non-blocking TeleOp usage.
  - Auto translation helpers remain in RUN_USING_ENCODER and taper speed off encoder deltas so commanded distances land consistently across different speed caps.
  - Translation taper floors for straight moves and twist-blended moves live in [`config/DriveTuning`](./config/DriveTuning.java) so crews can raise/lower the minimum speed without editing drivetrain code.
  - `stopAll()` alias keeps StopAll compatibility across modes.
- **Iterative Notes:** Motor direction fixes, IMU normalization, and settled-turn logic all surfaced in the file header to document drivetrain bring-up history.

### 🚀 Launcher ([`subsystems/Launcher.java`](./subsystems/Launcher.java))
- **Role:** Dual goBILDA 5202 flywheel control with RPM telemetry, AutoSpeed hooks, and manual overrides.
- **Highlights:**
  - PIDF constants piped from [`config/LauncherTuning`](./config/LauncherTuning.java) during initialization.
  - Works with [`control/LauncherAutoSpeedController`](./control/LauncherAutoSpeedController.java) and [`assist/AutoAimSpeed`](./assist/AutoAimSpeed.java) for AprilTag-based RPM targets.
  - `isAtSpeed(tolerance)` aligns with shared tolerances defined in [`config/SharedRobotTuning`](./config/SharedRobotTuning.java).
- **Iterative Notes:** Header documents transition from open-loop power to velocity control and integration testing for dual-motor current draw.

### ⚙️ Feed ([`subsystems/Feed.java`](./subsystems/Feed.java))
- **Role:** Timed feed motor routine with safety gating around launcher readiness.
- **Highlights:**
  - Tuned via [`config/FeedTuning`](./config/FeedTuning.java) to keep Auto and TeleOp cadence identical.
  - Provides StopAll-aware `halt()` to satisfy safety requirements.
  - Locks BRAKE zero-power behavior and RUN_WITHOUT_ENCODER mode before every command so the pusher holds position between cycles.
- **Iterative Notes:** Update history captures the gating of feed actions on `Launcher.isAtSpeed()` to prevent jams.

### 🌀 Intake ([`subsystems/Intake.java`](./subsystems/Intake.java))
- **Role:** Simple forward/reverse motor control with assist timers after feeds.
- **Highlights:**
  - Power constants read from [`config/IntakeTuning`](./config/IntakeTuning.java).
  - Supports jam-clearing reverse and integrates with StopAll routines.

### 🎮 Controller Bindings ([`input/ControllerBindings.java`](./input/ControllerBindings.java))
- **Role:** Centralized button/trigger/toggle tracking for both gamepads.
- **Highlights:**
  - `Pad`, `Btn`, and `Trigger` enums standardize references across TeleOp.
  - Toggle state persists frame-to-frame without manual bookkeeping in OpModes.
  - Works closely with rumble helpers to emit consistent haptic feedback.

### 🎯 Vision & Aim ([`vision/VisionAprilTag.java`](./vision/VisionAprilTag.java), [`vision/TagAimController.java`](./vision/TagAimController.java), [`assist/AutoAimSpeed.java`](./assist/AutoAimSpeed.java))
- **Role:** Provide AprilTag detections, convert pose to inches, and feed aim/twist corrections.
- **Highlights:**
  - `VisionAprilTag` wraps the FTC `VisionPortal`, enables the Driver Station live stream, applies alliance filtering, and latches Obelisk patterns through [`utils/ObeliskSignal`](./utils/ObeliskSignal.java).
  - `TagAimController` implements PD steering with tunables surfaced in [`config/TagAimTuning`](./config/TagAimTuning.java).
  - `AutoAimSpeed` unifies AprilTag distance-to-RPM mapping and aim assistance for both TeleOp and Autonomous while honoring [`config/AutoAimTuning`](./config/AutoAimTuning.java).

### 🕹 TeleOp ([`teleop/TeleOpAllianceBase.java`](./teleop/TeleOpAllianceBase.java), [`teleop/TeleOp_Blue.java`](./teleop/TeleOp_Blue.java), [`teleop/TeleOp_Red.java`](./teleop/TeleOp_Red.java))
- **Role:** Shared TeleOp implementation with alliance-specific wrappers that only override the `alliance()` hint.
- **Highlights:**
  - Handles AutoAim/AutoSpeed toggles, rumble notifications, StopAll latch, and auto-stop timer logic.
  - Initializes shared subsystems and config classes, ensuring `config/*` overrides propagate at runtime.
  - Scales translation while AutoAim is active and exposes manual RPM D-pad nudges whenever AutoSpeed is disabled **and manual lock is engaged**.
  - LB fire taps now honor the configured FeedStop lead/hold timing before any continuous streaming kicks in; LB holds only convert to continuous feed after the release window so single taps return to a one-shot cadence.
  - Every feed request performs a brief AutoAim nudge whenever a goal tag is visible—even if AutoAim is toggled off—then restores the driver’s AutoAim setting after the shot.
  - Triple-tapping the RB intake toggle latches the intake in reverse (power in `IntakeTuning`) until the next tap restores the saved intake state.
  - Exposes telemetry for drivetrain, launcher, and Obelisk signal states, including alliance-aware AprilTag distance and rumble prompts described in the [TeamCode README](./readme.md).

### 🛰 Odometry & AprilTag Fusion ([`odometry/Odometry.java`](./odometry/Odometry.java))
- **Role:** Provides a fused field pose for Auto and TeleOp using drive wheel deltas, IMU heading, and AprilTag goal detections.
- **Coordinate system:** (0, 0) at the human wall center with +X right and +Y toward the goals; offsets for the intake, launcher, and camera come from [`config/OdometryConfig`](./config/OdometryConfig.java).
- **Dashboard overlays:** FTC Dashboard now mirrors phone telemetry each loop using [`odometry/DecodeFieldDrawing`](./odometry/DecodeFieldDrawing.java) with the corrected +X/+Y transform, alliance-aware artifact rows (red/blue start, row Y lines, 5" spacing/radius), and triangular launch zones.
- **Vision use:** AprilTag corrections are blended with configurable weight/step limits whenever the red or blue goal tags are visible (tag poses live in `OdometryConfig.TAG_*_GOAL_*` and include XYZ+yaw); `computeVisionPose(...)` lets Auto/TeleOp seed from a tag during INIT, and `update(...)` keeps applying tag fixes during loops.
- **Pose seeding:** Autos call `setStartingPose(x, y, headingDeg)` during INIT (outside the builder chain) to declare the expected robot-center pose; BaseAuto also attempts a tag-based seed when a goal tag is visible before START.
- **Pose handoff:** `PoseStore` captures the fused pose at Auto stop so TeleOp can resume from the same origin if no tag is visible, surfacing a telemetry line when the saved pose seeds odometry; both modes still prefer live tag corrections when available and continue to blend goal-tag sightings during TeleOp driving.

### 🤖 Autonomous Framework ([`auto/BaseAuto.java`](./auto/BaseAuto.java), [`auto/Auto_*`](./auto))
- **Role:** Linear OpMode base plus alliance-specific routes (Human vs. Target starting positions).
- **Highlights:**
  - Reuses the same subsystems as TeleOp, including AutoAim/AutoSpeed, to reduce divergence.
  - Honors shared tunables from [`config/SharedRobotTuning`](./config/SharedRobotTuning.java) for drive caps, turn tolerances, and RPM readiness.
  - Calls `VisionAprilTag.observeObelisk()` during pre-start to cache Obelisk patterns before the buzzer, mirroring the workflow documented in [`teleop/TeleOpAllianceBase.java`](./teleop/TeleOpAllianceBase.java).
  - AutoSequence now includes `fireContinuous(label, timeMs, requireLock)` for timed continuous volleys that hold the FeedStop open and sustain launcher RPM.

### 🛑 StopAll System (Cross-cutting)
- **Role:** Ensures all actuators halt instantly when drivers press **Start** or when the optional timer expires.
- **Highlights:**
  - Implemented in `TeleOpAllianceBase` with explicit state latch and telemetry feedback.
  - Subsystems expose `stop()`/`halt()` helpers so the latch can zero motors without duplicating code.
  - TeleOp headers document recovery workflow (press **Start** again to resume) and the optional auto-stop timer mirrored in [`config/TeleOpDriverDefaults`](./config/TeleOpDriverDefaults.java).

---

## 🧱 Iterative Development Log (Highlights)

| Area | Issue Tracked in Headers | Resolution | Outcome |
|------|--------------------------|------------|---------|
| **Drivebase** | Encoder direction mismatches, IMU drift | Motor direction audit, heading reset utilities | Stable field-centric driving and reliable Auto paths |
| **Launcher** | RPM variance under load | Migrated to PIDF control, then tuned the AprilTag curve through [`config/AutoRpmConfig`](./config/AutoRpmConfig.java) and [`control/LauncherAutoSpeedController`](./control/LauncherAutoSpeedController.java) | Repeatable volleys under match battery sag |
| **Feed** | Premature firing | Locked firing to launcher readiness window | Cleaner shot cadence |
| **Intake** | Artifact jams | Added reverse toggle and intake assist timer | Faster cycle recovery |
| **TeleOp** | Duplicate button logic | Centralized [`ControllerBindings`](./input/ControllerBindings.java) + rumble cues | Simplified driver training |
| **Vision** | Range scaling errors | Converted meters→inches and tuned [`config/VisionTuning`](./config/VisionTuning.java) range scale | Accurate AutoSpeed seeds |
| **StopAll** | Post-buzzer safety | Latching Start button handler + optional timer | Compliance with match shutdown rules |

---

## 🔍 Lessons Learned

- **Centralized tunables prevent drift** – storing every parameter in `config/` keeps TeleOp and Auto synchronized even as students experiment.
- **Field-centric math demands calibration** – IMU mounting (`SharedRobotTuning.LOGO_DIRECTION/USB_DIRECTION`) and strafing compensation (`DriveTuning.STRAFE_CORRECTION`) should be validated together after every rebuild.
- **Vision aids should fail gracefully** – AutoAim retains last RPM and twists gently, so drivers can take over immediately when tags disappear.
- **StopAll builds driver trust** – a visible latch state and consistent recovery routine keep compliance simple during chaotic endgames.
- **Documentation accelerates onboarding** – maintaining headers, the Tunable Directory, and this Codex background file lets new developers absorb context without reading every class.

---

## 📚 References

- **FTC DECODE Competition Manual TU06 (2025–2026)**
- **FTC Team Update 00 (September 6, 2025)**
- **Indianola Robotics internal tuning logs & code reviews (2025 season)**
- [TeamCode Tunable Directory](./TunableDirectory.md) – authoritative configuration index for this codebase

---

*Prepared for Codex / Developer Context*  
*Indianola Robotics – FTC 2025–26 DECODE Season*
