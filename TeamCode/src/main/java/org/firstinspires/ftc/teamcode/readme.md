# Indianola Robotics – FTC 2025 Robot Code - DECODE Season

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
|----------|-----------|
| **Left Stick** | Drive (forward/back & strafe) |
| **Right Stick X** | Rotation |
| **Left Trigger** | Brake – reduces top speed |
| **Right Trigger** | Manual RPM control (only when manualSpeedMode = true) |
| **Left Bumper (LB)** | **Feed once (Feed subsystem)** |
| **Right Bumper (RB)** | **Toggle Intake On/Off** |
| **Right Stick Button (RS)** | **Toggle Aim‑Assist** |
| **Y / Triangle** | **Toggle Manual‑Speed Mode** |

### Gamepad 2 – Co‑Driver
| Control | Function |
|----------|-----------|
| **Left Bumper (LB)** | **Feed once (Feed subsystem)** |
| **Right Bumper (RB)** | **Toggle Intake On/Off** |
| **Y / Triangle** | **Toggle Manual‑Speed Mode** |
| *(RS / RT omitted)* | *(No joystick‑based controls on G2)* |

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
                            ├── auto/
                            │   ├── BaseAuto.java
                            │   ├── Auto_Red_Target.java
                            │   ├── Auto_Red_Human.java
                            │   ├── Auto_Blue_Target.java
                            │   └── Auto_Blue_Human.java
                            ├── vision/
                            │   ├── TagAimController.java
                            │   └── VisionAprilTag.java
                            └── input/
                                └── ControllerBindings.java
```

---

## File / Folder Guide

### **Alliance.java**
Enum for the current alliance (`RED` / `BLUE`). Used by TeleOp and Auto.

---

### **drive/Drivebase.java**
Encapsulates all drivetrain control logic.

**Provides**
- `drive(drive, strafe, twist)` — TeleOp robot‑centric control  
- `move(distanceInches, degrees, speed)` — encoder‑based translation  
- `turn(degrees, speed)` — IMU‑based rotation

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
| **Feed.java** | Times single‑ball feed into launcher. | `firePower`, `fireTimeMs`, `minCycleMs` |
| **Intake.java** | Toggles intake motor. | `powerOn` |

---

### **teleop/**
| File | Purpose |
|---|---|
| **TeleOpAllianceBase.java** | Shared TeleOp logic (sticks, braking, launcher/feed/intake, and AprilTag aim‑assist). |
| **TeleOp_Red.java** / **TeleOp_Blue.java** | Alliance‑specific wrappers around the base TeleOp. |

**Control Summary (Gamepad 1)**  
LB = Feed once RB = Intake toggle RS = Aim‑Assist Y = Manual‑Speed Mode

**Telemetry (always shown)**
- Alliance, BrakeCap, Intake state, ManualSpeed, RT, RPM Target/Actual  
- Aim Enabled, Tag Visible, Tag Heading (deg), **Tag Distance (inches)**

---

### **auto/**
| File | Purpose |
|---|---|
| **BaseAuto.java** | Common Auto init/teardown. |
| **Auto_*_Target.java** | Example “goal‑side” routes. |
| **Auto_*_Human.java** | Example human‑player routes. |

Each Auto uses `@Autonomous(..., preselectTeleOp="TeleOp - Red/Blue")`  
so the Driver Station preloads the correct TeleOp.

---

## Vision (AprilTags)

We use a USB webcam (“Webcam 1”) through VisionPortal + AprilTagProcessor  
to aim at the **alliance GOAL tag** and report heading/distance.

**Files**
- `vision/VisionAprilTag.java` – initializes VisionPortal, exposes a **compatibility getter** for detections across SDK versions, and supports **range scaling** to correct distance. Uses **640×480 MJPEG** for built‑in calibration and higher FPS.  
- `vision/TagAimController.java` – computes twist correction from tag **bearing** (deg) with simple PD control.

**TeleOp Integration**
- RS Button **toggles** Aim‑Assist ON/OFF (not hold).  
- When Aim‑Assist is **ON** and a tag is visible, TeleOp **overrides only twist** so drivers can still translate normally.  
- Telemetry **always** shows tag info: heading (deg) and distance (**inches**).

**Alliance Targeting**
- Blue GOAL = **ID 20**  
- Red  GOAL = **ID 24**

**Distance Calibration**
- Field tags are **8.125 in** squares.  
- If distance appears linearly off, set a **range scale**:  
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

## Quick Start for New Developers

1. Open project in **Android Studio** (FTC SDK 9.x+).  
2. To change controller bindings → edit `ControllerBindings.java`.  
3. To change drive tuning → adjust constants in `Drivebase.java`.  
4. To calibrate aim behavior → tune `TagAimController.kP` / `kD`.  
5. To add a new subsystem → create it under `/subsystems/` and initialize in `TeleOpAllianceBase.java`.  
6. Use Driver Station to select **TeleOp_Red** or **TeleOp_Blue**.

---

## Development Context

For maintainers, mentors, and future developers:
- **Robot Architecture:** Mecanum drivetrain with IMU heading control.  
- **Launcher:** Dual goBILDA 5202 6000‑RPM motors, closed‑loop PID.  
- **IMU Orientation:** Logo Up, USB Right.  
- **Vision:** `VisionAprilTag.java` → `TagAimController.java` for twist correction.  
- **Goal Selection:** Based on alliance color (Tag 20 = Blue, Tag 24 = Red).  
- **Control Flow:** `TeleOpAllianceBase` (input + control loop), `ControllerBindings` (button edges), subsystems (hardware logic).  
- **Telemetry:** Drive, launcher RPM, intake state, aim status, tag heading, tag distance.  
- **File Header Standard:** Every Java file includes structured comments (`FILE / LOCATION / PURPOSE / NOTES / METHODS`).  
- **Rule Reference:** Follow the **FTC 2025–2026 Competition Manual** and all **Team Updates** included with this project.

---

## Notes
- Vision system and telemetry are fully preserved from earlier builds.  
- Alliance selection determines tag target.  
- Webcam configured as “Webcam 1”.  
- Field rules and sizing per 2025–2026 DECODE manual.

---

### Revision History
**2025‑10‑22** – ControllerBindings integration finalized.  
**2025‑10‑23** – LB ↔ RB control swap; readme updated.  
**2025‑10‑24** – Integrated original readme content and expanded for DECODE season.

---

## Credits
Indianola Robotics – FTC Team 2025  
Mentor Support: *Jason Cross*  
Built on the official **FIRST Tech Challenge SDK**
