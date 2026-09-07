# BIOBUZZ robot foundation

Reusable mecanum, controller, webcam, and Limelight infrastructure from the
previous season is retained here. Game mechanisms, scoring behavior, field tags,
and routes are specific to DECODE and remain in the season release. All starter
OpModes are disabled until the new robot is configured.

Read the [foundation and tuning guide](../../../../../../../../docs/reusable-foundation.md)
for the active settings and the [V0 preseason notes](../../../../../../../../docs/biobuzz-preseason.md)
for preliminary season constraints. The [historical tuning reference](../../../../../../../../docs/decode-tuning-reference.md)
preserves every former configuration file, including settings not yet used by
the new runtime.

## First setup

1. Set motor names/directions, IMU name, and device names in `config/RobotConfig.java`.
   `MecanumDrive` provides basic driving without an IMU. The separate restored
   `Drivebase` encoder/IMU helpers require review of `DriveTuning` and
   `SharedRobotTuning`, then `CALIBRATION_CONFIRMED = true`.
2. Select `VisionConfig.SOURCE` (`WEBCAM` or `LIMELIGHT`). Configure the webcam
   profile in `VisionTuning` or Limelight pipeline/polling settings in
   `LimelightTuning`. Old C270 profiles and pipeline sampling values are retained
   as starting points, with explicit opt-in for calibration-dependent behavior.
3. Remove `@Disabled` from `teleop/VisionTest.java` for independent camera
   testing. It does not require a drivetrain. Status reports connection/pipeline
   or webcam controls. No wireless video/telemetry server is included.
4. Add new-season tag metadata in `VisionConfig.createTagLibrary()` and set
   `TARGET_TAG_ID` explicitly. The default library is empty and the target ID is
   unset. Unknown pose values are represented by NaN rather than an inherited
   distance correction or field transform.
5. Remove `@Disabled` from `teleop/BaseDriveTeleOp.java` when wheel directions
   are ready to test. Left stick moves, right X turns, LT slows, A stops while
   held, and Y toggles aim-feedback rumble. Vision is optional via
   `RobotConfig.VISION_ENABLED`. RB only provides aim assist when explicitly
   enabled in `VisionConfig` and the selected target has a fresh bearing.
6. Add autonomous actions to the disabled `auto/BaseAuto.java` after calibrating
   the new robot. The restored `Drivebase` supplies move/turn/moveWithTwist,
   timeout and stall handling, encoder access, and loop hooks. Keep resource
   cleanup in `finally` and do not reuse DECODE routes as BIOBUZZ routes.

## Organization

- `config`: active hardware, driver, motion, camera, Limelight, aiming and rumble settings.
- `input`: gamepad press/hold/toggle/trigger bindings and optional paddle readers.
- `drive`: basic wheel mixing plus calibrated encoder/IMU motion helpers.
- `vision`: interchangeable camera sources, target observations, pipeline sampling, and aiming.
- `utils`: driver feedback.
- `odometry`: pose container and handoff; no enabled field-pose fusion.
- `teleop`, `auto`: disabled starting OpModes for the new robot.

Both vision sources use positive-right bearing; basic drivetrain twist is
positive clockwise. Camera-derived distance is in meters when available. See
the foundation guide for autonomous heading conventions and known hardware
validation requirements.

## Recovering the DECODE robot

`master` and `decode-2025-2026-final-v2` contain the updated final DECODE code,
including the February 17 Target 9 modes and Human-route adjustments. The older
`DecodeFinal` and `decode-2025-2026-final` tags remain unchanged.

```sh
git worktree add ../FtcRobotController-DECODE decode-2025-2026-final-v2
```

The v2 tree matches the team's clean checkout at `69bb375`, also preserved by
`archive/decode/team-checkout-2026-02-17`. See the
[comparison report](../../../../../../../../docs/limelight-retirement.md).
