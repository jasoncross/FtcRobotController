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
- Shared across TeleOp and Auto.

### 🚀 Launcher ([`subsystems/Launcher.java`](./subsystems/Launcher.java))
- Dual 5202 flywheels under velocity PIDF.
- Integrated AutoSpeed logic for tag-based RPM.

### ⚙️ Feed ([`subsystems/Feed.java`](./subsystems/Feed.java))
- Coordinated with launcher readiness and StopAll safety.

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

---

## 🛰 Odometry & AprilTag Fusion ([`odometry/Odometry.java`](./odometry/Odometry.java))
- Fuses wheel odometry + IMU + vision.
- LL3A MegaTag1/2 results will become the primary pose correction source.
- P480 AprilTag positions are legacy-only and will be fully replaced this season.

---

## 🤖 Autonomous Framework ([`auto/BaseAuto.java`](./auto/BaseAuto.java))
- Shared Auto initialization, seeding, and aim/RPM helpers.
- LL3A corrections will drive long-range shot consistency and positional accuracy.

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
| **Vision** | P480 instability | **Migrated to Limelight 3A** | Reliable heading/distance |
| **TeleOp** | Button logic duplication | Centralized bindings | Cleaner driver workflow |

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
