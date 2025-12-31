# Indianola Robotics – FTC 2025 *DECODE* Season  
## Codex Context & Development Background

---

## 🧭 Project Overview

This `TeamCode` module contains the custom robot control stack for the **Indianola Robotics Octobots** in the **2025–2026 FTC Season: DECODE™ presented by RTX**. All code builds on the public FTC SDK that lives in the sibling `FtcRobotController` module.

Match strategy centers on consistent scoring of **ARTIFACTS** into **GOALS**, rapid interaction with **OBELISKS**, and building **PATTERNS** under the expansion and safety rules highlighted below. The robot architecture implemented in this branch provides:

- **Field-centric mecanum drivetrain** powered by [`drive/Drivebase.java`](./drive/Drivebase.java).
- **Dual flywheel launcher** with a distance-aware AutoSpeed curve managed by [`control/LauncherAutoSpeedController.java`](./control/LauncherAutoSpeedController.java).
- **Synchronized feed and intake subsystems** ([`subsystems/Feed.java`](./subsystems/Feed.java), [`subsystems/Intake.java`](./subsystems/Intake.java)).
- **Primary vision system: Limelight 3A AprilTag targeting**, defined in [`/vision/Limelight3A.md`](./vision/Limelight3A.md), feeding shared aim and RPM controllers such as [`assist/AutoAimSpeed.java`](./assist/AutoAimSpeed.java) and upcoming Limelight fusion helpers.
- **Limelight AUTO pipeline enforcement:** Autonomous now forces Limelight into an obelisk-observation pipeline during INIT motif checks and flips into the goal-aim pipeline (alliance-tag gated: BLUE 20 / RED 24) whenever scans/aiming begin, reasserting as needed during search loops so Autos are self-contained even without prior TeleOp prep.
- **Legacy P480 webcam pipeline (DEPRECATED)** – The P480-based AprilTag pipeline implemented in [`vision/VisionAprilTag.java`](./vision/VisionAprilTag.java) is retained *only* so existing Auto/TeleOp OpModes continue to compile this season.  
  **Codex must not generate new code using P480 / VisionPortal pipelines.**  
  **All new targeting, heading, distance, and pose code must use the Limelight 3A exclusively.**
- **Shared TeleOp and Auto frameworks** that reuse the same subsystems, tunables, and safety guards ([`teleop/TeleOpAllianceBase.java`](./teleop/TeleOpAllianceBase.java), [`auto/BaseAuto.java`](./auto/BaseAuto.java)).
- **StopAll latch with optional auto-stop timer** orchestrated inside [`teleop/TeleOpAllianceBase.java`](./teleop/TeleOpAllianceBase.java) for end-of-match compliance.

All OpModes run on centralized configuration tables ([`config/*.java`](./config)) so drivetrain geometry, launcher tuning, rumble envelopes, and driver defaults remain aligned between TeleOp and Autonomous. See the [TeamCode Tunable Directory](./TunableDirectory.md) for the authoritative list.

---

## ⚙️ Design Philosophy

Guided by the [TeleOp and subsystem headers](./teleop/TeleOpAllianceBase.java), development focused on:

1. **Reliability in both match phases** — encoder-driven translation, IMU-locked turns, and shared AutoAim/AutoSpeed loops keep TeleOp and Auto behavior identical.
2. **Subsystem modularity** — each mechanism exposes a clean API (`Drivebase.drive(...)`, `Launcher.setTargetRpm(...)`, `Feed.fireOnce(...)`) so TeleOp and Autonomous can evolve without forking logic.
3. **Driver usability** — the [`input/ControllerBindings`](./input/ControllerBindings.java) layer debounces buttons, tracks toggles, and feeds haptic cues through [`utils/RumbleNotifier`](./utils/RumbleNotifier.java).
4. **Safety** — StopAll latch zeroes drivetrain, launcher, feed, and intake instantly.
5. **Transparency for students** — consistent headers, inline docs, and this Codex context file ensure fast onboarding.

---

## 📘 Rule Background (Inline Highlights)

Key DECODE rules that shaped the system are reinforced throughout `TeamCode` comments:

> **R105 (TU00)** – Horizontal expansion limited to 18"×18", vertical to 38".  
>
> **G414 (TU06)** – Illegal expansion can lead to disablement.  
>
> **G501 (TU00)** – Up to eight DC motors and ten servos.  
>
> **R601 (TU00)** – Approved 12 V battery pack via main power switch.  
>
> **Section 9.10 (TU06)** – AprilTags on field structures are official localization references; vision-assisted alignment encouraged.  
>  
> **Limelight 3A is now our primary AprilTag system. P480 is deprecated.**

These constraints drive the emphasis on stable IMU turning, safe power distribution, and reliable AprilTag alignment.

---

## 🧩 Subsystem & File Context

### 🧭 Drivebase ([`drive/Drivebase.java`](./drive/Drivebase.java))
- Field/robot-centric mecanum with IMU-backed turning.
- Encoder-based translation with controlled tapering.
- Twist-enabled moves keep translation locked to the requested field heading while steering toward the target yaw, with encoder-derived distance tracking that remains accurate during simultaneous rotation.
- Shared across TeleOp and Auto.
 - Encoder distance math assumes 1× (non-quadrature) counts per revolution in `DriveTuning.TICKS_PER_REV`; only use 4× counts if the hub actually reports quadrature ticks.

#### 🚀 Movement speed constraints to watch
- `SharedRobotTuning.DRIVE_MAX_POWER` is the global ceiling for AutoSequence move speed; lowering it below 1.0 will cap every scripted drive even if individual steps request more. 【F:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/SharedRobotTuning.java†L71-L72】
- `DriveTuning.AUTO_MOVE_MIN_SPEED` and `AUTO_MOVE_WITH_TWIST_MIN_TRANS_SPEED` set the taper floors inside `Drivebase.move(...)` and `moveWithTwist(...)`, so overly low values can make the robot crawl as it finishes encoder-driven moves while higher floors keep it brisk near the target. 【F:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/DriveTuning.java†L34-L39】
- `SharedRobotTuning.TURN_TWIST_CAP` clamps twist authority for scans and blended drive/aim steps; large heading changes during translation may feel slower if this cap is tight. 【F:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/SharedRobotTuning.java†L70-L72】

### 🚀 Launcher ([`subsystems/Launcher.java`](./subsystems/Launcher.java))
- Dual 5202 flywheels under velocity PIDF.
- Integrated AutoSpeed logic for tag-based RPM.

### ⚙️ Feed ([`subsystems/Feed.java`](./subsystems/Feed.java))
- Coordinated with launcher readiness and StopAll safety.
- FeedStop homing is now queued until START so INIT stays motionless; the servo homes and parks at the blocking angle as soon as
  the match begins.
- FeedStop can open immediately, but the feed motor now waits for the launcher RPM window (with the shared settle time) before
  moving, and the FeedStop return timer starts when the feed motor actually begins so the gate does not close early.

### 🌀 Intake ([`subsystems/Intake.java`](./subsystems/Intake.java))
- Tuned power levels with jam-clearing logic.

### 🎮 Controller Bindings ([`input/ControllerBindings.java`](./input/ControllerBindings.java))
- Centralized button/toggle logic with rumble feedback.

---

## 🎯 Vision & Aim

### **Limelight 3A — Primary Vision System**
*(See [`docs/vision/Limelight3A.md`](./docs/vision/Limelight3A.md) for full details.)*

- Provides **heading (tx)**, **distance (3D pose)**, and **full-field localization (MegaTag1/MegaTag2)**.
- USB-only device; appears as `EthernetDevice` in DS config.
- Supports 90FPS pipelines, neural detection, Python pipelines, and built-in FTC field map.
- All new AutoAim, AutoSpeed, and OdometryFusion development must target the Limelight 3A pipeline.
- A new `VisionTargetProvider` abstraction fronts heading + distance; Limelight is now the default source while a legacy webcam wrapper exists only for fallback builds. `TagAimController` and `AutoAimSpeed` both consume the provider so aim PD and RPM gating share the same source. `BaseAuto` and TeleOp construct the provider (Limelight default, webcam fallback), Limelight latches obelisk motifs, and AutoSequence `visionMode(...)` steps no-op when Limelight is active to avoid webcam-only swaps.
- AUTO now applies alliance-only goal filtering with Limelight-side hysteresis (multi-frame acquire/loss counters and a short tx hold) so single-frame dropouts no longer flip between scan and aim; BaseAuto telemetry surfaces raw vs. smoothed visibility, held tx, lost-frame count, and pipeline index for verification.
- Limelight AutoAim now locks strictly to the alliance goal fiducial (BLUE 20 / RED 24) using that tag’s own tx/tz sample; global tx and obelisk sightings are excluded from twist control, locks expire after a brief **250 ms** stale window to stop aim when the goal vanishes, and heading now falls back to per-fiducial pose-derived tx when the Limelight omits tx fields for the goal tag.
- Limelight odometry fusion now filters botpose localization to a configurable goal-tag whitelist (default 20/24) using the fiducial override call so obelisk AprilTags never influence pose, hard-rejects any MT2 update that sees obelisk tags (with a debug override), and applies cautious/confirm correction clamps after long blind travel so reacquire converges without snapping while aim/RPM targeting stays alliance-specific through the Limelight priority ID. IMU-aligned seeding defines `headingOffsetDeg = wrap(seedHeading − imuYawAtSeed)` so fused heading stays continuous.
- FTC treats MegaTag2 as a **mode** (pipeline + yaw feed), so localization reads a single stable pose source (`LLResult.getBotpose()` by default) and reports whether MT2 is active via yaw freshness + tag count telemetry; a guarded `USE_LLRESULT_BOTPOSE_MT2` toggle exists only for SDK experiments.
- MegaTag2 stabilization continuously feeds the robot’s IMU yaw to Limelight every loop (0° toward +Y, CCW positive), and odometry rejects any vision pose outside ±90 in X/Y so fusion never seeds off-field.
- MegaTag2 yaw is fed each loop through LimelightHelpers’ FTC-safe static orientation calls (reflection on `SetRobotOrientation`/`setRobotOrientation` only—no WPILib NTCore), using the Limelight table name `LL_NT_NAME`. The `VisionDbg` line reports `yawOk`, `yawSent`, and `yawAgeMs` so crews can confirm the feed is fresh.
- Localization filters prefer LimelightHelpers’ `SetFiducialIDFiltersOverride`/`setFiducialIDFiltersOverride` (with Limelight3A fallback) so only goal tags (20/24) contribute to pose while aiming remains alliance-specific via `priorityid`. `VisionDbg` includes `fltOk` and `fltAgeMs` to confirm the filter is applied and recent.
- Limelight botpose streams are corner-origin; odometry shifts them to the FTC center-origin frame by subtracting 72 in from both axes after axis swap/sign correction (controlled by `APPLY_CENTER_SHIFT`) and before applying calibration offsets.
- Axis mapping is locked to `AXIS_SWAP_XY=true`, `X_SIGN=+1`, `Y_SIGN=-1`, and the corner→center shift must occur after axis swap/sign correction to stay in field coordinates.
- Bounds checks now evaluate the **final candidate pose** (post center-shift + offsets) against `FIELD_HALF_IN - BOUNDS_MARGIN_IN`, and telemetry reports the exact test values.
- Odometry now includes a dedicated distance scale (`ODOMETRY_DISTANCE_SCALE`) so Auto drive distances stay physical while pose calibration remains adjustable.
- WPILib NetworkTables are intentionally avoided in FTC builds; reflection is scoped to LimelightHelpers only for SDK compatibility.
- TeleOp now appends a single `VisionDbg` telemetry line showing yaw feed status/age, localization filter status/age, MT2 expected/active state, pose source, raw meters, transformed raw pose (pre-offset), accepted vs. fused pose, and the last rejection reason so crews can diagnose pose drift quickly. It also reports the localization tag whitelist (`locIDs`), current tag allow-list status (`tidOk`), per-frame tag participation (`visibleIDs`, `obeliskSeen`, `obeliskIDs`), and correction mode/step (`mode`, `step`) with optional `rawBlue`/`rawRed` fields gated by `DEBUG_VERBOSE_VISION`.
- Calibration update: `VisionConfig.LimelightFusion.X_OFFSET_IN`/`Y_OFFSET_IN` now default to `0.0` after applying the corner→center transform; adjust only after validating the new frame alignment.
- INIT now runs a Limelight pipeline auto-selection pass in both TeleOp and Auto. The selector reads the tunable `LIMELIGHT_PIPELINE_PROFILES` list, ensures the Limelight stream is started, switches through each pipeline (with settle + multi-frame sampling delays), and scores them using hit-count qualification: **alliance goal hits → opposing goal hits → none** (obelisk tags are ignored for selection). If START occurs mid-selection, the non-blocking state machine continues running post-start until it locks a winner or the max-selection timeout expires; START no longer forces fallback, selection continues through START without resets, and the fallback pipeline index is tunable.
- Successful selections (best-rank hits) now update the **Last Known Good** pipeline (index + description) so tag-less autos can reuse the most recently validated profile.
- If no relevant tags are seen (best-rank = 0), the selector prefers the last-known-good pipeline when enabled, logs a non-urgent end-group line like `LL AUTOSELECT: MEMORY FALLBACK -> <idx> - <desc> (reason=no_tags)`, and still reports `LL Profile: <index> - <description>`.
- If no last-good is available (or memory is disabled), the selector falls back to `PIPELINE_FALLBACK_INDEX` and surfaces the **urgent** first-line banner: `LL AUTOSELECT FALLBACK -> <idx> - <desc> (reason=...)`.
- Timeout fallbacks can optionally use last-known-good when `PIPELINE_USE_LAST_GOOD_ON_TIMEOUT` is true, and last-good persistence across RC restarts is controlled by `PIPELINE_PERSIST_LAST_GOOD_ENABLED`.
- `AutoAimTuning.INVERT_AIM_TWIST` flips aim-generated twist before it hits the drivebase in both TeleOp and Auto so clockwise/counter-clockwise rotation stays consistent once hardware direction is verified, without touching manual stick twist.
- Long-shot aim windows now bias **RED** toward negative bearings and **BLUE** toward positive bearings once the distance cutover engages, keeping long volleys on the correct side of center in both TeleOp and Auto.

### **Legacy P480 AprilTag Pipeline (DEPRECATED)**  
- Implemented in [`vision/VisionAprilTag.java`](./vision/VisionAprilTag.java).  
- Retained strictly for backward compatibility with existing OpModes.  
- **Codex must not generate or extend code using the P480 pipeline.**  
- **All future vision-related code is to be built exclusively around the Limelight 3A.**  
- Auto/TeleOp aim logic previously tied to P480 is being migrated to LL3A-based heading + pose.

---

## 🕹 TeleOp ([`teleop/TeleOpAllianceBase.java`](./teleop/TeleOpAllianceBase.java))
- Central TeleOp layer: AutoAim toggles, rumble cues, StopAll latch, intake modes.
- Now integrates LL3A heading/distance where available.
- Legacy P480 preview screens remain temporarily for testing but are no longer used for targeting.
- Temporary AutoAim shot assists triggered by feed holds now unwind cleanly after the stream ends, restoring the driver’s AutoAim
  toggle instead of leaving AutoAim latched on when continuous fire is released.

---

## 🛰 Odometry & AprilTag Fusion ([`odometry/Odometry.java`](./odometry/Odometry.java))
- Uses the FTC-standard field-center frame (0,0 in the middle; +X right, +Y toward targets) with IMU-only heading and mecanum wheel deltas. Start-pose seeding aligns an IMU heading offset so Auto and TeleOp share consistent heading.
- Optionally fuses Limelight botpose XY (MegaTag2 preferred) with two-phase gating: tight outlier rejection while tracking, a larger window after `reacquireAfterMs`, stability-counted cautious/confirm correction clamps, and motion gates on speed/turn rate. Yaw is never fused.
- Webcam pose fusion has been removed entirely; Limelight is the only vision source allowed to influence odometry.

---

## 🤖 Autonomous Framework ([`auto/BaseAuto.java`](./auto/BaseAuto.java))
- Shared Auto initialization, seeding, and aim/RPM helpers.
- LL3A corrections will drive long-range shot consistency and positional accuracy.
- Start-pose seeding now aligns the IMU heading offset and INIT telemetry reports the seed pose, IMU yaw, and any vision override reason.
- Auto dashboard updates are now driven from a single cached odometry update each loop so the field overlay stays in lockstep with motion without double-updating; Auto loops should call `updateOdometryPose()` once per iteration and pass the cached pose into `updateStatusWithPose(...)`.
- Auto saves the final fused pose (including heading) to `PoseStore` on stop/cleanup, and TeleOp restores that pose on init to preserve heading continuity.
- AutoSequence `move(...)` steps now treat the heading argument as **relative to the robot’s current facing** so translation
  vectors stay robot-centric even after mid-match heading changes; the builder resolves the absolute heading internally and
  reports both values in telemetry.
- Aim/launcher prep loops exit as soon as their readiness goals are satisfied, using timeouts only as fallbacks so sequences
  proceed immediately once a step is complete.

---

## 🛑 StopAll System (Cross-cutting)
- Safety-critical: all motors zero immediately.  
- Enabled during TeleOp and can be triggered automatically.

---

## 🧱 Iterative Development Log (Highlights)

| Area | Issue | Resolution | Outcome |
|------|--------|-------------|---------|
| **Drivebase** | IMU drift | Normalization + reset utilities | Stable heading |
| **Launcher** | RPM load variance | PIDF tuning + tag-based AutoSpeed | Consistent volleys |
| **Vision** | P480 instability | **Migrated to Limelight 3A** (with TeleOp pipeline alias retained post split) | Reliable heading/distance |
| **TeleOp** | Button logic duplication | Centralized bindings | Cleaner driver workflow |
| **Maintenance** | Odometry build warning | Restored missing List import after vision guard updates | Clean FTC SDK compile |

---

## 🔍 Lessons Learned

- Centralizing tunables prevents drift.  
- Field-centric math demands consistent IMU handling.  
- **Limelight 3A dramatically increases long-range shot consistency** due to stable pose + heading.  
- Legacy P480 pipelines require too much exposure/gain tuning—now fully deprecated.  
- StopAll latch builds driver trust and simplifies endgame safety.  
- Documentation + headers accelerate student learning and Codex development.

---

## 📚 References

- FTC 2025–26 DECODE Competition Manual TU06  
- FTC Team Update 00  
- Internal tuning logs & field test videos  
- [`docs/vision/Limelight3A.md`](./docs/vision/Limelight3A.md) – authoritative technical guide for Limelight integration  
- [`TunableDirectory.md`](./TunableDirectory.md)

---

*Prepared for Codex / Developer Context*  
*Indianola Robotics – FTC 2025–26 DECODE Season*
