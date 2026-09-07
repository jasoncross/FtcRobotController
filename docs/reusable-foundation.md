# Reusable robot infrastructure

The initial BIOBUZZ cleanup removed too much reusable code. This revision restores
the controller, drive, and vision infrastructure while leaving DECODE scoring
mechanisms, routes, field IDs, and launcher behavior in the season release.

## Active configuration

| Area | Configuration | Retained capability |
| --- | --- | --- |
| Hardware | `RobotConfig` | Motor names/directions, drive power cap, IMU name, webcam name, optional vision |
| Driver inputs | `ControllerTuning`, `ControllerBindings` | Two gamepads, press/hold/toggle bindings, trigger threshold 0.5, analog triggers, optional paddle readers, deadband, strafe/turn scaling, trigger braking floor 0.25 |
| Feedback | `TeleOpRumbleTuning`, `RumbleNotifier` | Aim feedback threshold 2.5 degrees, 0.10-0.65 strength, 120-200 ms pulses, 120-350 ms cooldown, toggle confirmation pattern |
| Autonomous drive | `DriveTuning`, `SharedRobotTuning`, `Drivebase` | Encoder motion, IMU turns, translation with heading changes, cruise/taper behavior, stall detection, loop hooks, wheel encoder access |
| Camera | `VisionTuning`, `AprilTagVision` | 480p practice/event and 720p C270 profiles, decimation, decision-margin filtering, exposure/gain/white-balance controls, explicit intrinsics and SDK tag library |
| Limelight | `VisionConfig`, `LimelightTuning`, `LimelightTargetProvider` | Native SDK polling, pipeline selection, fresh fiducial bearings, device status, explicit yaw-feed API |
| Pipeline selection | `LimelightPipelineSelector` | Nonblocking settling/sampling, fresh-frame deduplication, qualifying tag list, hit threshold, timeout, fallback pipeline |
| Target aiming | `TagAimTuning`, `TagAimController` | Retained frame-delta PD gains 0.02/0.003, clamp 0.6, deadband 1.5 degrees, bearing offset, explicit target ID, stale-target rejection |
| Pose handoff | `FieldPose`, `PoseStore` | Reusable pose container and Auto-to-TeleOp handoff; no inherited field coordinates or enabled localization fusion |

Configuration files are in
[`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config).
Most restored settings are editable Java fields; no Dashboard/configuration
server has been restored. Camera/provider settings are applied when the device
is constructed; edit, rebuild, and re-INIT after changing them.

## Camera profiles

`VisionTuning.PRESET` selects one of these profiles. The historical values are
retained for a C270, but must be checked against the camera actually installed.

| Preset | Resolution | Decimation | Minimum decision margin | Exposure / gain | Intrinsics fx, fy, cx, cy |
| --- | --- | --- | --- | --- | --- |
| SDK_DEFAULT | 640x480 | 2.0 | 10 | SDK automatic | SDK calibration |
| C270_P480_PRACTICE | 640x480 | 2.0 | 10 | 6 ms / 85 | 690, 690, 320, 240 |
| C270_P480_EVENT | 640x480 | 2.0 | 10 | 2 ms / 50 | 690, 690, 320, 240 |
| C270_P720 | 1280x720 | 2.2 | 24 | 7 ms / 85 | 1380, 1035, 640, 360 |

`SDK_DEFAULT` is initially selected. Set `USE_MANUAL_CONTROLS` and/or
`USE_CALIBRATED_INTRINSICS` explicitly after selecting and checking a historical
profile. Manual settings are attempted after the camera reaches STREAMING;
telemetry reports which controls succeeded. Unsupported camera controls do not
silently appear successful. LiveView is disabled by default.

`VisionConfig.createTagLibrary()` is the shared place to add new tag metadata.
It remains empty until the new field is known. Webcam detections with no pose
metadata have unknown bearing/distance; they cannot drive aim assist. Distances
from a configured webcam library are explicitly in meters, with no old `0.03`
range multiplier applied. The current Limelight adapter reports bearing and ID,
but leaves distance unknown pending verified camera/map setup.

## Limelight bring-up

1. Select `VisionConfig.Source.LIMELIGHT` and set `LIMELIGHT_NAME` to the device's
   configured hardware name. Start with `LimelightTuning.PIPELINE_INDEX` set to
   a pipeline that is actually installed on the device.
2. Enable the independent `VisionTest` OpMode to inspect connection status,
   pipeline, IDs, and bearings. It needs no motors.
3. Optional auto-selection retains the historical starting values: pipeline
   slots 0/1/2, 250 ms settle, 6 samples, 60 ms sample interval, 5 target hits,
   5-second timeout, and fallback 0. It is disabled until the slots are reviewed.
   Empty `QUALIFYING_TAG_IDS` allows any fiducial to qualify a pipeline; it does
   not choose a game target for aiming. Frames from the previous pipeline and
   repeated frames do not count toward qualification.
4. Set `TARGET_TAG_ID` explicitly before using aim assist. There are no inherited
   goal/obelisk tags. Selection timing is bounded from initiation; START does
   not silently extend the timeout. Last-good pipeline persistence is not
   restored, avoiding stale choices from a different robot's pipeline layout.

The adapter uses FTC SDK `Limelight3A` directly, including
`updateRobotOrientation` for an explicitly supplied yaw. It does not use the old
reflection/NetworkTables helper, write a field map, or fuse a field pose into
odometry. The final team robot had pose fusion disabled too.

## Driver and autonomous controls

The disabled `BaseDriveTeleOp` starter uses left stick to move, right stick X
to turn, left trigger to reduce speed, A to stop while held, and Y to toggle
aim-feedback rumble. If `ENABLE_AIM_ASSIST` is explicitly enabled, holding RB
uses the configured target's bearing. Missing/stale/unsolved targets produce
zero aim twist; translation stays under driver control. All wheel commands
remain capped by `RobotConfig.DRIVE_POWER_LIMIT`.

`Drivebase` restores the old encoder/IMU helpers separately from the basic
`MecanumDrive`. `DriveTuning.CALIBRATION_CONFIRMED` is false until the new wheel
geometry, motor directions, and IMU mounting are checked. Historical effective
wheel diameter 4.098 inches, 537.7 ticks/revolution, strafe correction 1.15,
turn gains 0.012/0.003, taper and stall settings remain available. The effective
diameter is not a claim that a nominal 96 mm wheel is physically 4.098 inches.

The restored helpers use the new base's signs: positive drive is forward,
positive strafe/twist are right/clockwise, and IMU headings/relative `turn()`
angles are counterclockwise. `move()` headings are clockwise relative to the
robot; `moveWithTwist()` translation headings are clockwise relative to the
zero-yaw field frame, while its target heading is IMU counterclockwise.
Unlike the old robot, no hidden forward-power inversion is retained. Each
blocking helper has a configurable 10-second timeout and motor cleanup in
`finally`. A finishing turn after translation has its own bounded timeout.
These helpers require physical calibration/testing before routes are built.

`BaseAuto` remains an empty disabled template. Restoring motion infrastructure
does not restore DECODE autonomous routes into BIOBUZZ.

## Historical settings retained as reference

[DECODE tuning reference](decode-tuning-reference.md) contains the exact contents
of every old `config/*.java` file from team commit `69bb375`. This includes
camera distortion coefficients, brightness normalization/equalization, exposure
nudging, health thresholds, visibility hysteresis, odometry fusion parameters,
controller preferences, and mechanism tuning that are not active in this base.
Those settings have not disappeared; they are clearly distinguished from knobs
that the new runtime actually consumes. The complete implementations remain in
`decode-2025-2026-final-v2` for deliberate reuse.

## Verification

On September 7, 2026, both the DECODE v2 and restored BIOBUZZ debug builds passed.
All 10 JUnit tests passed, as did the standalone 9,261-input mecanum check.

Build with `./gradlew --no-daemon assembleDebug :TeamCode:testDebugUnitTest`.
The tests exercise controller edges, trigger/bumper independence, stale-target
handling, aim signs/clamps, pipeline sampling/fallback, field-coordinate math,
and slow-drive scaling. The standalone `MecanumMixerCheck` also checks 9,261
combined stick inputs. These checks do not replace testing motors, IMU mounting,
exposure controls, and Limelight pipelines on hardware.
