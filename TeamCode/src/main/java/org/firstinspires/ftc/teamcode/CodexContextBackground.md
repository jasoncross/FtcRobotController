# Current architecture context

This describes the BIOBUZZ foundation, not the final DECODE robot. Coding
instructions live in the repository's [AGENTS.md](../../../../../../../../AGENTS.md).
See [readme.md](readme.md) for setup and [TunableDirectory.md](TunableDirectory.md)
for active configuration. The detailed [foundation guide](../../../../../../../../docs/reusable-foundation.md)
and [historical tuning reference](../../../../../../../../docs/decode-tuning-reference.md)
separate reusable capability from calibration and game behavior.

## Ownership and data flow

| Component | Responsibility and callers |
| --- | --- |
| `BaseDriveTeleOp` | Creates basic drivetrain, gamepad bindings, rumble/aim controllers, and optional vision. Updates vision during INIT without driving. During RUN, reads controls, selects the configured target, optionally applies aim twist, and commands drive. Stops motors/rumble and closes vision in `finally`. |
| `ControllerBindings` | Edge, hold, toggle and analog-trigger callbacks across two gamepads. Call `update` each RUN loop. Multiple bindings can coexist on one button; registered paddle readers are optional. Game-specific default mappings were removed. |
| `MecanumDrive` / `MecanumMixer` | Basic robot-centric drive with normalized wheel powers, hardware-configured directions, and the configured power cap. Trigger braking scales the cap after normalization. No IMU required. |
| `Drivebase` | Separate retained encoder/IMU helper layer for calibrated autonomous motion. Not instantiated by the current BaseAuto or basic TeleOp. Constructor requires `CALIBRATION_CONFIRMED`; blocking helpers check active state, use timeouts and retained stall logic, and stop motors in `finally`. Do not construct both drive owners for the same hardware in one OpMode. |
| `VisionFactory` / `VisionTargetProvider` | Device selection through a common lifecycle/status/observation interface. Device providers do not own drive or scoring decisions. |
| `AprilTagVision` | SDK VisionPortal/AprilTagProcessor. Applies selected resolution, decimation, margin filtering, explicit tag library and optional intrinsics. Attempts manual exposure/gain/white-balance controls once streaming, reporting results. Closes the portal on exit. |
| `LimelightTargetProvider` | Native SDK Limelight3A polling and pipeline control, fresh fiducial ID/bearing observations and connection status. Stops polling on exit. Optional explicit yaw-feed API is not automatically driven by the current TeleOp. |
| `LimelightPipelineSelector` | Nonblocking settle/sample/choose/fallback process. Counts distinct frames only from the requested pipeline. Uses a bounded selection lifetime and no persistent last-good configuration. |
| `TargetObservation` / `TagAimController` | Timestamped ID, bearing and optional distance. Aim uses the explicit ID, rejects missing/stale/unsolved observations, applies frame-delta PD/deadband/clamp, and resets on loss. |
| `RumbleNotifier` | Nonblocking heading-error feedback with configurable envelope. TeleOp Y toggles feedback and plays the configured confirmation pattern. |
| `FieldPose` / `PoseStore` | Caller-defined pose and copied Auto-to-TeleOp handoff. Utilities retained for future localization; no current OpMode performs pose fusion through them. |
| `BaseAuto` | Disabled, no-motion LinearOpMode template. There are no BIOBUZZ route variants or fluent `sequence()` implementation yet. |
| `VisionTest` | Independent disabled vision bring-up OpMode. Runs either provider without drive hardware, including updates during INIT. |

## Units and signs

- Basic drive: positive forward, positive strafe right, positive twist clockwise.
- Shared vision bearing: positive right. Webcam FTC bearing is negated at the
  provider boundary; Limelight horizontal target angle already uses this sign.
- Vision distance: camera-to-tag meters when available, otherwise NaN. The
  current Limelight adapter intentionally leaves distance unknown. No old
  range correction multiplier or field transform is applied.
- Observation timestamps: monotonic nanoseconds; age limit is configured in ms.
- IMU yaw and `Drivebase.turn` angles: degrees counterclockwise.
- `Drivebase.move` direction: clockwise from robot-forward.
- `moveWithTwist` translation direction: clockwise from zero-yaw field-forward;
  its target heading is an absolute IMU counterclockwise heading.
- Encoder geometry and `FieldPose` X/Y: inches. FieldPose's origin is chosen
  by its caller, not hard-coded to a DECODE wall or field corner.

## Current defaults and limits

All three starter OpModes are `@Disabled`. Basic drive is capped at 0.4, vision
in TeleOp is off, source is webcam, tag ID is unset (-1), and automatic aiming
is off. The tag library is empty until explicitly populated. Webcam observations
without metadata cannot provide an aiming bearing or range.

Old C270 profiles, drive gains/geometry, and rumble/aim envelopes are available
as baselines. Manual camera controls, explicit intrinsics, Limelight auto-select,
and calibrated Drivebase construction each require their documented configuration.
There is no Dashboard or third-party network video/telemetry server. The dated
[V0 notes](../../../../../../../../docs/biobuzz-preseason.md) record preliminary
rules; do not infer game timing, field tag IDs, or expansion limits from DECODE.

## Season boundary

`master` and `decode-2025-2026-final-v2` retain the final DECODE tree, including
Target 9 modes. Launcher, feed/intake sequencing, AutoRPM, obelisk detection,
DECODE field drawing and scoring paths are historical features, not dependencies
of the current architecture. Recover useful logic deliberately through that tag;
do not restore entire old OpMode classes merely to obtain a helper.
