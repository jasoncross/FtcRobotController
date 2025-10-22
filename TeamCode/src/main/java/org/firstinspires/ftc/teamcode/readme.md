# Indianola Robotics – FTC 2025 Robot Code

## Overview
This project contains the full robot control code for the **Indianola Robotics Team** (FTC 2025 season).  
The robot uses a **mecanum drivetrain**, **dual flywheel launcher**, **feed motor**, **intake**, and **AprilTag-based targeting**.

All TeleOps and Autos share the same subsystem classes for consistency and easy maintenance.

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
                            │   └── OpModeShim.java
                            ├── subsystems/
                            │   ├── Launcher.java
                            │   ├── Feed.java
                            │   └── Intake.java
                            ├── teleop/
                            │   ├── TeleOpAllianceBase.java
                            │   ├── TeleOp_Red.java
                            │   └── TeleOp_Blue.java
                            └── auto/
                            │   ├── BaseAuto.java
                            │   ├── Auto_Red_Target.java
                            │   ├── Auto_Red_Human.java
                            │   ├── Auto_Blue_Target.java
                            │   └── Auto_Blue_Human.java
                            ├── vision/
                            │   ├── TagAimController.java
                            │   └── VisionAprilTag.java
```

---

## File / Folder Guide

### **Alliance.java**
Enum for the current alliance (`RED` / `BLUE`). Used by TeleOp and Auto.

---

### **drive/Drivebase.java**
Encapsulates all drivetrain control logic.

**Provides**
- `drive(drive, strafe, twist)` — TeleOp robot-centric control  
- `move(distanceInches, degrees, speed)` — encoder-based translation  
- `turn(degrees, speed)` — IMU-based rotation

**Key Tunables**
| Variable | Meaning |
|---|---|
| `WHEEL_DIAMETER_IN` | Real wheel diameter (inches) |
| `TICKS_PER_REV` | Encoder ticks per motor rev |
| `STRAFE_CORRECTION` | Multiplier for accurate strafing (≈1.10–1.25 typical) |
| `TURN_KP`, `TURN_KD` | Turn PID gains |
| `TURN_TOLERANCE_DEG` | Stop threshold for turns |
| `TURN_SETTLE_TIME` | Time (s) within tolerance before finishing turn |

---

### **util/OpModeShim.java**
Adapter to allow Iterative OpModes to reuse components that expect a LinearOpMode context.

---

### **subsystems/**
| File | Purpose | Key Tunables |
|---|---|---|
| **Launcher.java** | Controls dual flywheels. | `bottomRPM`, `topRPM`, `atSpeedToleranceRPM` |
| **Feed.java** | Times single-ball feed into launcher. | `firePower`, `fireTimeMs`, `minCycleMs` |
| **Intake.java** | Toggles intake motor. | `powerOn` |

---

### **teleop/**
| File | Purpose |
|---|---|
| **TeleOpAllianceBase.java** | Shared TeleOp logic (sticks, braking, launcher/feed/intake, and AprilTag aim-assist). |
| **TeleOp_Red.java** / **TeleOp_Blue.java** | Alliance-specific wrappers around the base TeleOp. |

**Control Summary (Gamepad 1)**
| Control | Action |
|---|---|
| Left stick | Forward/back + strafe |
| Right stick X | Rotate |
| Left trigger | Brake (reduces top speed) |
| Right trigger | Manual launch RPM (if manual mode ON) |
| X / A | Feed one ball |
| Left Bumper | Toggle intake ON/OFF |
| Right Bumper | **Toggle Aim‑Assist** (maintains aim at alliance tag) |
| Triangle / Y | Toggle manual launch‑speed mode |

**Telemetry (always shown)**
- Alliance, BrakeCap, Intake state, ManualSpeed, RT, RPM Target/Actual  
- Aim Enabled, Tag Visible, Tag Heading (deg), **Tag Distance (inches)**

---

### **auto/**
| File | Purpose |
|---|---|
| **BaseAuto.java** | Common Auto init/teardown. |
| **Auto_*_Target.java** | Example “goal-side” routes. |
| **Auto_*_Human.java** | Example human-player routes. |

Each Auto uses `@Autonomous(..., preselectTeleOp="TeleOp - Red/Blue")` so the Driver Station preloads the correct TeleOp.

---

## Vision (AprilTags)

We use a USB webcam (“Webcam 1”) through VisionPortal + AprilTagProcessor to aim at the **alliance GOAL tag** and report heading/distance.

**Files**
- `vision/VisionAprilTag.java` – initializes VisionPortal, exposes a **compatibility getter** for detections across SDK versions, and supports **range scaling** to correct distance. Uses **640×480 MJPEG** for built‑in calibration and higher FPS.
- `vision/TagAimController.java` – computes twist correction from tag **bearing** (deg) with simple PD control.

**TeleOp Integration**
- Right Bumper **toggles** Aim‑Assist ON/OFF (not hold).  
- When Aim‑Assist is **ON** and a tag is visible, TeleOp **overrides only twist** so drivers can still translate normally.
- Telemetry **always** shows tag info: heading (deg) and distance (**inches**).

**Alliance Targeting**
- Blue GOAL = **ID 20**  
- Red  GOAL = **ID 24**

**Distance Calibration**
- Field tags are **8.125 in** squares. If distance appears linearly off, set a **range scale**:
  ```java
  vision.setRangeScale(trueMeters / measuredMeters);
  ```
- TeleOp displays **inches**: `inches = meters × 39.3701`.

**Tuning (Aim)**
| Setting | Where | Default | Notes |
|---|---|---|---|
| `kP` | `TagAimController` | 0.02 | Raise for faster response |
| `kD` | `TagAimController` | 0.003 | Raise to reduce oscillation |
| Twist Clamp | `TagAimController` | ±0.6 | Cap rotational authority while strafing |
| Deadband | `TagAimController` | 1.5° | Prevents hunting near 0° error |
| `rangeScale` | `VisionAprilTag` | 1.0 | Apply after 1‑point calibration |
| `cameraResolution` | `VisionAprilTag` | 640×480 | Uses built‑in calibration |
| `streamFormat` | `VisionAprilTag` | MJPEG | Higher FPS; ignored if unsupported |

**Hardware Setup**
- Plug webcam into the Control Hub’s **blue USB 3.0** port.
- Add as **“Webcam 1”** in the active Robot Configuration.

---

## Development Tips
1. Build/deploy frequently and test one subsystem at a time.  
2. Verify motor directions after firmware/SDK changes.  
3. IMU orientation: **Logo UP, USB RIGHT**.  
4. Tune strafing on carpet (adjust `STRAFE_CORRECTION`).  
5. Use telemetry for RPM, heading, distance (inches), and aim state.

---

## Credits
Indianola Robotics – FTC Team 2025  
Mentor Support: *Jason Cross*  
Built on the official **FIRST Tech Challenge SDK**
