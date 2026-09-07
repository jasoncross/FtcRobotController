# BIOBUZZ tunable directory

Current source defaults as of 2026-09-07. Settings live in [config/](config/). Historical DECODE values are retained as starting points, not verified BIOBUZZ calibration. See [the reusable foundation](../../../../../../../../docs/reusable-foundation.md) for setup and coordinate conventions.

Edit source, rebuild/redeploy, and re-INIT to apply changes reliably. This base does not provide a live tuning dashboard. `Drivebase` caches geometry, gains and taper settings in static constants when the class loads; re-INIT alone does not refresh those caches. Device/source/profile settings are captured during construction; rumble settings are copied during TeleOp setup. Some controls, aim gains and freshness thresholds are read during updates, but this is not a promise that every setting updates live.

Units and intended ranges below are tuning guidance unless enforcement is explicitly stated. Verify camera/device-supported values and chassis calibration before enabling features.

## [ControllerTuning](config/ControllerTuning.java)

| Setting | Default | Units / constraints | Consumer / effect |
| --- | --- | --- | --- |
| `TRIGGER_EDGE_THRESH` | `0.5` | Normalized trigger input | ControllerBindings trigger press edge |
| `STICK_DEADBAND` | `0.05` | Normalized stick magnitude | BaseDriveTeleOp zeros small stick inputs |
| `SLOWEST_SPEED` | `0.25` | Fraction, clamped 0–1 | BaseDriveTeleOp full-trigger speed floor relative to drive cap |
| `STRAFE_SCALE` | `1.0` | Multiplier | BaseDriveTeleOp lateral stick scale |
| `TURN_SCALE` | `1.0` | Multiplier | BaseDriveTeleOp manual twist scale |

## [DriveTuning](config/DriveTuning.java)

| Setting | Default | Units / constraints | Consumer / effect |
| --- | --- | --- | --- |
| `CALIBRATION_CONFIRMED` | `false` | Boolean; must be true to construct Drivebase | Explicit review gate for historical chassis calibration |
| `AUTO_ACTION_TIMEOUT_SEC` | `10.0` | Seconds; finite and positive at construction | Drivebase bounded action loops; a finishing turn has its own timeout |
| `WHEEL_DIAMETER_IN` | `4.098` | Inches; positive geometry required | Drivebase encoder conversion; historical effective diameter |
| `TICKS_PER_REV` | `537.7` | Encoder counts per motor output revolution | Drivebase encoder conversion |
| `GEAR_RATIO` | `1.0` | Motor output revolutions per wheel revolution | Drivebase encoder conversion |
| `STRAFE_CORRECTION` | `1.15` | Multiplier | Drivebase empirical lateral correction |
| `TURN_KP` | `0.012` | Power per degree | Drivebase IMU turn proportional gain |
| `TURN_KD` | `0.003` | Power per (degree/second) | Drivebase time-based turn derivative gain |
| `TURN_TOLERANCE_DEG` | `1.0` | Degrees | Drivebase acceptable turn error |
| `TURN_SETTLE_TIME_SEC` | `0.15` | Seconds | Drivebase time inside turn tolerance |
| `AUTO_MOVE_MIN_SPEED` | `0.30` | Normalized power | Drivebase move taper floor |
| `AUTO_MOVE_WITH_TWIST_MIN_TRANS_SPEED` | `0.30` | Normalized power | Drivebase moveWithTwist translation floor |
| `AUTO_MOVE_TAPER_START_FRACTION` | `0.10` | Fraction of travel distance | Drivebase end-of-move deceleration window |
| `AUTO_MOVE_TWIST_SCALE` | `0.60` | Multiplier | Drivebase heading-hold twist scaling |
| `AUTO_MOVE_MAX_TWIST` | `0.35` | Normalized absolute power | Drivebase heading-hold twist clamp |
| `AUTO_ENABLE_STALL_EXIT` | `true` | Boolean | Drivebase translation stall exit |
| `AUTO_STALL_VELOCITY_EPSILON` | `0.5` | Inches/second | Drivebase low-velocity stall threshold |
| `AUTO_STALL_POSITION_EPSILON` | `0.25` | Inches of progress per sample | Drivebase insufficient-progress threshold |
| `AUTO_STALL_TIME_MS` | `400.0` | Milliseconds | Drivebase minimum stalled interval before exit |

## [LimelightTuning](config/LimelightTuning.java)

| Setting | Default | Units / constraints | Consumer / effect |
| --- | --- | --- | --- |
| `POLL_HZ` | `30` | Polls/second; device-supported rate | LimelightTargetProvider polling setup |
| `PIPELINE_INDEX` | `0` | Device pipeline slot | LimelightTargetProvider fixed pipeline when AUTO_SELECT is false |
| `AUTO_SELECT` | `false` | Boolean | LimelightTargetProvider enables nonblocking pipeline selection |
| `PIPELINES` | `{0, 1, 2}` | Array of installed pipeline slots | LimelightPipelineSelector candidate order; verify on device |
| `SETTLE_MS` | `250` | Milliseconds | Selector waits after pipeline switch |
| `SAMPLE_COUNT` | `6` | Distinct fresh frames per candidate | Selector sampling budget |
| `SAMPLE_INTERVAL_MS` | `60` | Milliseconds | Selector minimum sampling interval |
| `MAX_SELECTION_MS` | `5000` | Milliseconds | Selector overall deadline |
| `MIN_TARGET_HITS` | `5` | Qualifying frames | Selector successful-candidate threshold |
| `FALLBACK_INDEX` | `0` | Device pipeline slot | Selector fallback when no candidate qualifies |
| `QUALIFYING_TAG_IDS` | `{}` | Tag ID array; empty accepts any fiducial for pipeline scoring | LimelightTargetProvider pipeline qualification; does not select an aim target |

## [RobotConfig](config/RobotConfig.java)

| Setting | Default | Units / constraints | Consumer / effect |
| --- | --- | --- | --- |
| `FRONT_LEFT` | `"front_left"` | Hardware configuration name | MecanumDrive / Drivebase front-left motor |
| `FRONT_RIGHT` | `"front_right"` | Hardware configuration name | MecanumDrive / Drivebase front-right motor |
| `BACK_LEFT` | `"back_left"` | Hardware configuration name | MecanumDrive / Drivebase back-left motor |
| `BACK_RIGHT` | `"back_right"` | Hardware configuration name | MecanumDrive / Drivebase back-right motor |
| `IMU_NAME` | `"imu"` | Hardware configuration name | Drivebase IMU |
| `FRONT_LEFT_DIRECTION` | `DcMotorSimple.Direction.REVERSE` | Motor direction enum | MecanumDrive / Drivebase front-left direction |
| `FRONT_RIGHT_DIRECTION` | `DcMotorSimple.Direction.FORWARD` | Motor direction enum | MecanumDrive / Drivebase front-right direction |
| `BACK_LEFT_DIRECTION` | `DcMotorSimple.Direction.REVERSE` | Motor direction enum | MecanumDrive / Drivebase back-left direction |
| `BACK_RIGHT_DIRECTION` | `DcMotorSimple.Direction.FORWARD` | Motor direction enum | MecanumDrive / Drivebase back-right direction |
| `DRIVE_POWER_LIMIT` | `0.4` | Normalized power cap, intended 0–1 | MecanumDrive starter TeleOp output limit |
| `VISION_ENABLED` | `false` | Boolean | BaseDriveTeleOp optional vision initialization; VisionTest is independent |
| `WEBCAM_NAME` | `"Webcam 1"` | Hardware configuration name | AprilTagVision webcam lookup |

## [SharedRobotTuning](config/SharedRobotTuning.java)

| Setting | Default | Units / constraints | Consumer / effect |
| --- | --- | --- | --- |
| `LOGO_DIRECTION` | `RevHubOrientationOnRobot.LogoFacingDirection.UP` | REV hub mounting orientation enum | Drivebase IMU initialization; historical mounting |
| `USB_DIRECTION` | `RevHubOrientationOnRobot.UsbFacingDirection.RIGHT` | REV hub mounting orientation enum | Drivebase IMU initialization; historical mounting |

## [TagAimTuning](config/TagAimTuning.java)

| Setting | Default | Units / constraints | Consumer / effect |
| --- | --- | --- | --- |
| `KP` | `0.02` | Power per degree | TagAimController bearing proportional gain |
| `KD` | `0.003` | Power per degree of frame-to-frame error change | TagAimController derivative; not time-normalized |
| `CLAMP_ABS` | `0.6` | Absolute power, magnitude capped at 1 | TagAimController twist limit before drivetrain mixing |
| `DEADBAND_DEG` | `1.5` | Degrees; absolute magnitude used | TagAimController on-target zero-output window |

## [TeleOpRumbleTuning](config/TeleOpRumbleTuning.java)

| Setting | Default | Units / constraints | Consumer / effect |
| --- | --- | --- | --- |
| `AIM_RUMBLE_ENABLED` | `true` | Boolean | BaseDriveTeleOp initial RumbleNotifier enabled state; Y toggles |
| `AIM_THRESHOLD_DEG` | `2.5` | Degrees | RumbleNotifier aim feedback window |
| `AIM_STRENGTH_MIN` | `0.10` | Normalized strength, intended 0–1 | RumbleNotifier strength at window edge |
| `AIM_STRENGTH_MAX` | `0.65` | Normalized strength, intended 0–1 | RumbleNotifier strength near zero error |
| `AIM_PULSE_MIN_MS` | `120` | Milliseconds | RumbleNotifier shortest pulse at window edge |
| `AIM_PULSE_MAX_MS` | `200` | Milliseconds | RumbleNotifier longest pulse near zero error |
| `AIM_COOLDOWN_MIN_MS` | `120` | Milliseconds | RumbleNotifier shortest cooldown at window edge |
| `AIM_COOLDOWN_MAX_MS` | `350` | Milliseconds | RumbleNotifier longest cooldown near zero error |
| `TOGGLE_STRENGTH` | `0.8` | Normalized strength, intended 0–1 | BaseDriveTeleOp toggle confirmation bursts |
| `TOGGLE_STEP_MS` | `120` | Milliseconds | BaseDriveTeleOp confirmation burst duration |
| `TOGGLE_GAP_MS` | `80` | Milliseconds | BaseDriveTeleOp gap between confirmation bursts |

## [VisionConfig](config/VisionConfig.java)

| Setting | Default | Units / constraints | Consumer / effect |
| --- | --- | --- | --- |
| `SOURCE` | `Source.WEBCAM` | WEBCAM or LIMELIGHT | VisionFactory provider selection |
| `LIMELIGHT_NAME` | `"limelight"` | Hardware configuration name | LimelightTargetProvider device lookup |
| `TARGET_TAG_ID` | `-1` | Integer tag ID; -1 means unset | TeleOp / VisionTest explicit target selection |
| `ENABLE_AIM_ASSIST` | `false` | Boolean | BaseDriveTeleOp right-bumper aim enable; requires usable selected target |
| `MAX_TARGET_AGE_MS` | `120` | Milliseconds | Providers, aim controller and rumble reject stale observations |
| `AIM_BEARING_OFFSET_DEG` | `0` | Degrees, positive right | Aim and feedback subtract this desired bearing from observation |

## [VisionTuning](config/VisionTuning.java)

| Setting | Default | Units / constraints | Consumer / effect |
| --- | --- | --- | --- |
| `PRESET` | `Preset.SDK_DEFAULT` | VisionTuning.Preset enum | AprilTagVision captured profile selection |
| `USE_MANUAL_CONTROLS` | `false` | Boolean | AprilTagVision attempts profile exposure/gain/white-balance controls |
| `USE_CALIBRATED_INTRINSICS` | `false` | Boolean | AprilTagVision supplies profile intrinsics only when explicitly enabled |
| `LIVE_VIEW_ENABLED` | `false` | Boolean | AprilTagVision local portal preview at construction |
| `SDK_DEFAULT` | `new Profile(640, 480, 2.0f, 10, 0, 0, false, 0, 0, 0, 0)` | Profile constructor tuple; parameter order below | AprilTagVision default profile; manual controls and custom intrinsics remain off |
| `C270_P480_PRACTICE` | `new Profile(640, 480, 2.0f, 10, 6, 85, true, 690, 690, 320, 240)` | Profile constructor tuple; parameter order below | AprilTagVision when selected by PRESET; C270 profiles are historical |
| `C270_P480_EVENT` | `new Profile(640, 480, 2.0f, 10, 2, 50, true, 690, 690, 320, 240)` | Profile constructor tuple; parameter order below | AprilTagVision when selected by PRESET; C270 profiles are historical |
| `C270_P720` | `new Profile(1280, 720, 2.2f, 24, 7, 85, true, 1380, 1035, 640, 360)` | Profile constructor tuple; parameter order below | AprilTagVision when selected by PRESET; C270 profiles are historical |

## Webcam profile parameters

Profile tuples above use this exact constructor order. `SDK_DEFAULT` still configures resolution, decimation and decision-margin filtering; it leaves camera controls/intrinsics to the SDK unless the corresponding flags are enabled.

| Setting | Default | Units / constraints | Consumer / effect |
| --- | --- | --- | --- |
| `Profile.width` | `640` (SDK_DEFAULT) | Pixels | Portal resolution width |
| `Profile.height` | `480` (SDK_DEFAULT) | Pixels | Portal resolution height |
| `Profile.decimation` | `2.0f` (SDK_DEFAULT) | Detector decimation factor | AprilTag processor |
| `Profile.minDecisionMargin` | `10` (SDK_DEFAULT) | Detector decision-margin score | Detection quality filter |
| `Profile.exposureMs` | `0` (SDK_DEFAULT) | Milliseconds; positive for manual exposure | Optional exposure control |
| `Profile.gain` | `0` (SDK_DEFAULT) | Camera-specific gain units | Optional gain control |
| `Profile.lockWhiteBalance` | `false` (SDK_DEFAULT) | Boolean | Optional manual white-balance mode |
| `Profile.fx` | `0` (SDK_DEFAULT) | Pixels; requires calibration | Optional focal length x |
| `Profile.fy` | `0` (SDK_DEFAULT) | Pixels; requires calibration | Optional focal length y |
| `Profile.cx` | `0` (SDK_DEFAULT) | Pixels; requires calibration | Optional principal point x |
| `Profile.cy` | `0` (SDK_DEFAULT) | Pixels; requires calibration | Optional principal point y |

## Tag metadata and historical references

`VisionConfig.createTagLibrary()` returns an empty library by default. Populate verified tag sizes/metadata before expecting solved webcam bearings/ranges. Limelight currently supplies bearing/ID observations, not range or field-pose fusion. Pipeline qualification IDs are separate from the explicitly selected aim target.

Rumble pulse durations above describe the consumer behavior: longer pulses and longer cooldowns occur near zero error. The existing pulse comments in `TeleOpRumbleTuning` reverse that relationship; follow `RumbleNotifier.update()` when interpreting them.

Old scoring mechanisms and their configuration are archived in [the DECODE tuning reference](../../../../../../../../docs/decode-tuning-reference.md) and DECODE tags. They are not active BIOBUZZ settings.
