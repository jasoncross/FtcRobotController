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

### Gamepad 1 – Driver
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
| **D-pad Left/Right** | **- / + 50 RPM** while TEST MODE is enabled (applies immediately) |
| **D-pad Down** | **Disable TEST MODE** and **STOP** launcher |

### Gamepad 2 – Co-Driver
| Control | Function |
|---|---|
| **Left Bumper (LB)** | **Feed once** (with **Intake Assist** if Intake is OFF) |
| **Right Bumper (RB)** | **Toggle Intake On/Off** |
| **Y / Triangle** | **Toggle AutoSpeed** (mirrors G1) |
| *(RS / RT omitted)* | *(No joystick-based controls on G2)* |

**Startup defaults:**  
`AutoSpeed = OFF`, `AutoAim = OFF`, `Intake = OFF` (editable in `TeleOpAllianceBase.java`).

---

## Project Layout
```
TeamCode/
└── src/
    └── main/
        └── java/
            └── org/
                └── firstinspires/
                    └── ftc/
                        └── teamcode/
                            Alliance.java
                            ├── drive/
                            │   └── Drivebase.java
                            ├── util/
                            │   └── RumbleNotifier.java
                            ├── subsystems/
                            │   ├── Launcher.java
                            │   ├── Feed.java
                            │   └── Intake.java
                            ├── teleop/
                            │   ├── TeleOpAllianceBase.java
                            │   ├── TeleOp_Red.java
                            │   └── TeleOp_Blue.java
                            ├── auto/
                            │   ├── BaseAuto.java
                            │   ├── Auto_Red_Target.java
                            │   ├── Auto_Red_Human.java
                            │   ├── Auto_Blue_Target.java
                            │   └── Auto_Blue_Human.java
                            ├── vision/
                            │   ├── TagAimController.java
                            │   └── VisionAprilTag.java
                            ├── control/
                            │   └── LauncherAutoSpeedController.java
                            └── input/
                                └── ControllerBindings.java
```

---

## TeleOp Behaviors & Tunables

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
- **Eject (B/Circle):** runs launcher at `ejectRpm = 300 RPM` for `ejectTimeMs = 300 ms`, feeds once, then restores previous RPM.  

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

## Tunable Reference (by Functional Area)

| Functional Area | Variable | Default | Purpose | File |
|---|---|---|---|
| **Drivebase** | `slowestSpeed` | 0.25 | Lowest speed cap when braking (trigger fully held) | TeleOpAllianceBase.java |
| **Launcher / AutoSpeed** | `rpmTop` | 6000 | Maximum launcher speed (RPM) | TeleOpAllianceBase.java |
| | `rpmBottom` | 0 | Minimum idle RPM in manual mode | TeleOpAllianceBase.java |
| | `InitialAutoDefaultSpeed` | 2500 | Launcher RPM before first tag fix | TeleOpAllianceBase.java |
| | `autoNearDistIn` | 24.0 | Near distance (in) for RPM mapping | TeleOpAllianceBase.java |
| | `autoNearRpm` | 1000 | RPM at near distance | TeleOpAllianceBase.java |
| | `autoFarDistIn` | 120.0 | Far distance (in) for RPM mapping | TeleOpAllianceBase.java |
| | `autoFarRpm` | 4500 | RPM at far distance | TeleOpAllianceBase.java |
| | `autoSmoothingAlpha` | 0.15 | AutoRPM smoothing factor (0–1) | TeleOpAllianceBase.java |
| **AutoAim / Vision** | `autoAimLossGraceMs` | 4000 | Time allowed (ms) to regain tag before disabling AutoAim | TeleOpAllianceBase.java |
| | `aimRumbleDeg` | 2.5 | Degrees from target center where aim rumble starts | TeleOpAllianceBase.java |
| | `kP` | 0.02 | Proportional gain for aim correction | TagAimController.java |
| | `kD` | 0.003 | Derivative gain for aim correction | TagAimController.java |
| | `twistClamp` | ±0.6 | Twist limit while aiming | TagAimController.java |
| | `deadband` | 1.5° | Ignore heading error below this | TagAimController.java |
| **Feed / Intake / Eject** | `firePower` | 0.9 | Power level during feed cycle | Feed.java |
| | `fireTimeMs` | 200 | Feed motor duration (ms) | Feed.java |
| | `minCycleMs` | 300 | Cooldown between feed cycles | Feed.java |
| | `powerOn` | 0.8 | Intake motor power | Intake.java |
| | `DEFAULT_INTAKE_ENABLED` | false | Intake state at TeleOp start | TeleOpAllianceBase.java |
| | `intakeAssistMs` | 250 | Time to run intake when assisting feed | TeleOpAllianceBase.java |
| | `ejectRpm` | 300 | Launcher RPM used during eject | TeleOpAllianceBase.java |
| | `ejectTimeMs` | 300 | Duration (ms) of eject phase | TeleOpAllianceBase.java |
| **Haptics / Rumble** | `togglePulseStrength` | 0.8 | Intensity of toggle rumble | TeleOpAllianceBase.java |
| | `togglePulseStepMs` | 120 | Duration of each rumble pulse | TeleOpAllianceBase.java |
| | `togglePulseGapMs` | 80 | Gap between double-pulse steps | TeleOpAllianceBase.java |
| | `aimRumbleMinStrength` | 0.10 | Min rumble strength for aim feedback | TeleOpAllianceBase.java |
| | `aimRumbleMaxStrength` | 0.65 | Max rumble strength for aim feedback | TeleOpAllianceBase.java |
| | `aimRumbleMinPulseMs` | 120 | Min pulse length (ms) for aim rumble | TeleOpAllianceBase.java |
| | `aimRumbleMaxPulseMs` | 200 | Max pulse length (ms) for aim rumble | TeleOpAllianceBase.java |
| | `aimRumbleMinCooldownMs` | 120 | Min cooldown (ms) between pulses | TeleOpAllianceBase.java |
| | `aimRumbleMaxCooldownMs` | 350 | Max cooldown (ms) between pulses | TeleOpAllianceBase.java |

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
- **2025‑10‑22** – Initial DECODE TeleOp base with AutoSpeed & AutoAim integration.  
- **2025‑10‑23** – Controller rumble feedback added; Intake assist logic implemented.  
- **2025‑10‑24** – AutoAim grace window (4 s) added with auto‑disable pulse.  
- **2025‑10‑25** – Eject function implemented; readme fully synchronized with code.  

---

## Credits
Indianola Robotics – FTC Team 2025  
Mentor Support: *Jason Cross*  
Built on the official **FIRST Tech Challenge SDK**  

---

© Indianola Robotics · DECODE Season (2025–2026)
