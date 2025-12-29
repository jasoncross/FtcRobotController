# 📘 Limelight 3A – Vision Integration Guide
**LOCATION:** `TeamCode/vision/Limelight3A.md`  
**PURPOSE:** Define how the Indianola Robotics codebase uses the Limelight 3A for heading, distance, and robot pose. Provide Codex with complete hardware/software context for correct subsystem integrations.  
**NOTES:** Complements (not replaces) VisionAprilTag.md and Odometry.md.

---

# 1. Overview

The **Limelight 3A** is a USB-connected, FTC-native vision processor. It replaces our USB webcam + OpenCV pipeline and provides reliable AprilTag detection and 3D localization directly to FTC Java via the Limelight3A SDK.

The LL3A is our **primary source for heading (tx), distance (pose), and pose correction** in auto and TeleOp. Odometry now fuses
only Limelight XY (MegaTag2 preferred) with bounded corrections; yaw remains IMU-only and webcam pose fusion is disabled.

**New abstraction (2025-12-11):** `vision/VisionTargetProvider` now defines the heading+distance contract used by TeleOp/Auto. `LimelightTargetProvider` implements this interface using Limelight tx + botpose (MT2 when available) while `WebcamLegacyTargetProvider` wraps the deprecated VisionPortal pipeline strictly for compatibility. Limelight remains the default source, and TeleOp plus BaseAuto now construct the provider based on `VisionConfig.VISION_SOURCE` (Limelight default, webcam legacy fallback) before feeding aim + launcher helpers. When Limelight is active it also latches Obelisk (21/22/23) tags directly from LL3A results so motif memory and telemetry remain live without the webcam stack.

---

# 2. Hardware Summary

- **Power/Data:** USB-C → Control Hub USB 3.0 port  
- **Power Draw:** 4W  
- **Sensor:** OV5647 (640×480 @ 90FPS)  
- **Field of View:** 54.5° horizontal, 42° vertical  
- **Mounting:** M3/M4 threaded, #10 thru-holes  
- **Illumination:** No built-in LEDs  
- **Status LED:**  
  - Slow blink = no target  
  - Fast blink = target detected  
- **Notes:** Appears as an “EthernetDevice’’ in DS robot config when connected by USB.

---

# 3. Software Summary (LimelightOS)

- Browser configuration: http://limelight.local:5801  
- 10 hot-swappable pipelines  
- Pipeline types include:  
  - 2D AprilTag  
  - 3D pose (MegaTag1)  
  - 3D pose + IMU fusion (MegaTag2)  
  - Blob tracking  
  - Neural detection/classification (CPU)  
  - Custom Python pipelines (OpenCV 4.10, numpy)  
- Built-in tools: MJPEG streaming, Charuco calibration, FTC field map  
- Performance:  
  - 20 FPS @ 1280×960  
  - 50 FPS @ 640×480  
  - 90 FPS color pipelines

---

# 4. How We Use Limelight Data

Codex must understand these three core outputs:

## 4.1 Heading (tx)
- Horizontal angle offset to target, in degrees
- Used for shooter auto-aim and drivetrain rotation alignment
- Consumed primarily by LimelightAimHelper and Drivebase PID turn routines
- AUTO applies Limelight-side hysteresis (multi-frame acquire/loss counters and a short tx hold) before feeding `TagAimController` so goal locks stay steady through single-frame flickers; telemetry surfaces raw/smoothed visibility, held tx, lost-frame count, and pipeline index for verification.

## 4.2 Distance
Primary source: **3D AprilTag pose data (botpose MT2 preferred)**
- Computes planar distance from the Limelight botpose XY to the alliance goal tag, scaled by `VisionConfig.LIMELIGHT_RANGE_SCALE`.
- Secondary fallback: **area (ta)** is no longer used now that botpose is available.
- Used to calculate shooter RPM, feed timing, and auto positional offsets

## 4.3 Robot Pose (MegaTag1 & MegaTag2)
- MegaTag1: 3D localization without IMU fusion
- MegaTag2: 3D localization fused with robot IMU yaw (still ignored for odometry heading)
- Used for odometry correction in autos/TeleOp as XY-only inputs (no yaw fusion)
- Consumed by odometry with two-phase gating (tight outlier rejection while tracking, larger reacquire window after `reacquireAfterMs`), per-step clamps, and speed/turn-rate motion gates to prevent snapping.
- Localization filters fiducials to goal tags **20/24** only (obelisk tags never feed `botpose`/`botpose_MT2`).
- MegaTag2 is stabilized by feeding the robot’s IMU yaw to Limelight every loop (0° toward +Y, CCW positive).
- Vision fusion rejects any Limelight pose outside ±90 in X/Y to keep pose seeding on-field.

---

# 5. FTC Integration Steps

1. Connect LL3A to Control Hub USB 3.0 port  
2. In DS: Configure Robot → Scan → Detect “EthernetDevice” → rename to `limelight`  
3. Initialize in code using hardwareMap:
   ```java
   private Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
