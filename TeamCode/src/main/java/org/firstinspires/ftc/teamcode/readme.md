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
                            |   ├── Auto_Red_Human.java
                            |   ├── Auto_Blue_Target.java
                            |   └── Auto_Blue_Human.java
                            ├── vision/
                            |   ├── TagAimController.java
                            │   └── VisionAprilTag.java
```

---

## File / Folder Guide

### **Alliance.java**
Simple enum indicating the current alliance (`RED` / `BLUE`).  
Used throughout to mirror autonomous routes and tag targeting.

---

### **drive/Drivebase.java**
Encapsulates all drivetrain control logic.  
Provides:
- `drive(drive, strafe, twist)` — TeleOp robot-centric control  
- `move(distanceInches, degrees, speed)` — encoder-based translation  
- `turn(degrees, speed)` — IMU-based rotation  

**Tune These:**
| Variable | Meaning |
|-----------|----------|
| `WHEEL_DIAMETER_IN` | Real wheel diameter (inches) |
| `TICKS_PER_REV` | Encoder ticks per motor rev |
| `STRAFE_CORRECTION` | Multiplier for accurate strafing (1.10–1.25 typical) |
| `TURN_KP`, `TURN_KD` | Turn PID gains |
| `TURN_TOLERANCE_DEG` | Stop threshold for turns |
| `TURN_SETTLE_TIME` | Time (s) within tolerance before finishing turn |

---

### **util/OpModeShim.java**
Adapter that lets OpModes using the **Iterative (OpMode)** style  
reuse subsystems expecting a **LinearOpMode** (like Drivebase).  
No tuning required.

---

### **subsystems/**
| File | Purpose | Key Tunables |
|------|----------|--------------|
| **Launcher.java** | Controls dual flywheel motors. | `bottomRPM`, `topRPM`, `atSpeedToleranceRPM` |
| **Feed.java** | Controls `FeedMotor` that pushes a ball into flywheels. | `firePower`, `fireTimeMs`, `minCycleMs` |
| **Intake.java** | Toggles intake motor on/off. | `powerOn` |

---

### **teleop/**
| File | Purpose |
|------|----------|
| **TeleOpAllianceBase.java** | Shared TeleOp code (sticks, triggers, feed, intake, launcher logic). |
| **TeleOp_Red.java** | Red-alliance TeleOp (inherits base). |
| **TeleOp_Blue.java** | Blue-alliance TeleOp (inherits base). |

**Control Summary (Gamepad 1):**
| Control | Action |
|----------|--------|
| Left stick | Forward/back + strafe |
| Right stick X | Rotate |
| Left trigger | Brake mode (reduces speed) |
| Right trigger | Manual launch speed (if manual mode ON) |
| X (Cross) | Fire one ball (FeedMotor) |
| L2 / LB | Toggle intake ON/OFF |
| R2 / RB | Aim assist toward AprilTag *(future)* |
| Triangle / Y | Toggle manual launch-speed mode ON/OFF |

Telemetry displays alliance, intake state, flywheel RPM target, and throttle cap.

---

### **auto/**
| File | Purpose | Notes |
|------|----------|-------|
| **BaseAuto.java** | Common auto setup and cleanup. |
| **Auto_Red_Target.java** | Red-side “goal” route example. |
| **Auto_Red_Human.java** | Red-side human-player route. |
| **Auto_Blue_Target.java** | Blue-side “goal” route example. |
| **Auto_Blue_Human.java** | Blue-side human-player route. |

Each Auto uses the annotation  
```java
@Autonomous(..., preselectTeleOp="TeleOp - Red/Blue")
```  
so the correct TeleOp automatically preloads on the Driver Station.

---

## Tuning Checklist
| Area | Variable | Notes |
|-------|-----------|-------|
| **Drive** | `STRAFE_CORRECTION` | Adjust until a 24" strafe moves ~24". |
|  | `TURN_KP`, `TURN_KD` | Tune for crisp turns without oscillation. |
| **Launcher** | `bottomRPM`, `topRPM` | Adjust for motor and wheel configuration. |
| **Feed** | `fireTimeMs` | Long enough to fully feed one ball, no more. |
| **Intake** | `powerOn` | Adjust to maintain control without stalling. |

---
## Vision (AprilTags)

We use a single USB webcam (“Webcam 1”) via VisionPortal + AprilTagProcessor to aim at the **alliance GOAL tag**.

**Files**
- `vision/VisionAprilTag.java` – initializes VisionPortal, returns the best detection for a requested tag ID.
- `vision/TagAimController.java` – computes twist (rotation) correction from tag bearing.

**TeleOp integration**
- `teleop/TeleOpAllianceBase.java`:
  - Right Bumper **toggles** Aim-Assist ON/OFF (not hold).
  - While Aim-Assist is **ON** and the alliance tag is visible, TeleOp **overrides only twist** to keep the robot aimed at the tag. Forward/back and strafe remain fully driver-controlled.
  - Telemetry adds: aim enabled, tag visible, tag heading (deg), tag distance (m).
  - All **existing telemetry** (intake, RPM, throttle cap, etc.) remains unchanged.

**Alliance targeting**
- Blue uses tag ID **20**; Red uses tag ID **24**.

**Tuning**
- In `TagAimController`: start with `kP=0.02`, `kD=0.003`. Increase `kP` for faster response; increase `kD` to reduce oscillation. Twist is clamped to ±0.6 and deadband is 1.5°.

**Notes**
- Plug webcam into the Control Hub’s blue USB 3.0 port and name it **"Webcam 1"** in the active configuration.


---

## Development Tips
1. **Build/Deploy frequently** – test one subsystem at a time.  
2. **Verify wheel directions** after moving to the new Drivebase (Forward/Strafe/Turn).  
3. **Calibrate IMU orientation** (Logo UP, USB FORWARD).  
4. **Tune constants on carpet** – friction affects strafing and turning.  
5. **Use telemetry** heavily while debugging (speed, distance, angles).  

---

## Credits
Indianola Robotics – FTC Team 2025  
Mentor Support: *Jason Cross*  
Built on the official **FIRST Tech Challenge SDK**
