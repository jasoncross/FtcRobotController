# BIOBUZZ robot foundation

This package starts the 2026–2027 robot with mecanum driving and webcam AprilTag
vision. It contains no DECODE scoring mechanisms, paths, goal IDs, field poses,
launcher curves, or robot calibration. All three example OpModes are `@Disabled`
until the hardware configuration is reviewed.

Read the [BIOBUZZ V0 preseason notes](../../../../../../../../docs/biobuzz-preseason.md)
before adding mechanisms or vision services. In particular, R704 restricts
robot Wi-Fi streaming to permitted Driver Station traffic and prohibits services
such as FTC Dashboard and continuous wireless video. Keep image processing
onboard and use Driver Station telemetry. V0 leaves game timing, field/tag data,
and expansion limits pending; do not restore those values from DECODE.

## First setup

1. Edit `config/RobotConfig.java` to match the Robot Controller configuration.
   The starter names are `front_left`, `front_right`, `back_left`, `back_right`,
   and `Webcam 1`. Review each motor direction on the new chassis.
2. Enable `teleop/BaseDriveTeleOp.java` by removing `@Disabled`. First verify
   wheel directions with the robot lifted. The initial power limit is 0.4.
   Left stick controls forward/strafe, right stick X rotates, and holding A
   stops the drive. Driving is robot-centric and requires no IMU or odometry.
3. Enable `teleop/VisionTest.java` to test the camera independently of the
   drivetrain. Confirm the configured resolution works with the selected camera.
   Set `VISION_ENABLED` to true only when you also want vision in the drive OpMode.
4. Supply an explicit `AprilTagLibrary` to `AprilTagVision` when the new tag
   IDs, physical sizes, and field placement are known. The default library is
   intentionally empty: it reports IDs and pixel centers, with no tag pose
   metadata. It does not use the SDK's DECODE `getCurrentGameTagLibrary()`.
   Calibrate camera intrinsics for the chosen resolution and camera mounting
   before adding distance estimation, field localization, or aiming.
5. Build new autonomous actions in `auto/BaseAuto.java`. It currently performs
   no movement. Add encoder/odometry calibration, bounded action loops, and
   stop handling before adding routes.

## Organization

- `config`: hardware names and initial settings for the new robot.
- `drive`: normalized mecanum power calculation and motor ownership.
- `vision`: camera lifecycle and AprilTag observations; no game decisions.
- `teleop`: driver control and independent vision bring-up.
- `auto`: autonomous starting point.

`BaseDriveTeleOp` stops the motors and closes the camera in `finally`;
`VisionTest` closes the camera with try-with-resources. Keep resource ownership
and cleanup explicit when adding mechanisms. The vision foundation uses the
FTC SDK webcam API; it is not a Limelight implementation. Add a separate
adapter if the team selects Limelight again.

## Recovering the DECODE robot

The annotated `decode-2025-2026-final` tag contains the merged season histories
and exactly matches the team's existing `DecodeFinal` file snapshot. To browse
or build the old robot without disturbing new work:

```sh
git worktree add ../FtcRobotController-DECODE decode-2025-2026-final
```

All removed mechanisms, vision fusion, motion helpers, calibration, and season
documentation remain available there. Recover individual ideas deliberately;
their old settings describe the DECODE robot.

The latest discovered team checkout is slightly newer: the tag
`archive/decode/team-checkout-2026-02-17` preserves two added Target 9 autonomous
modes and two Human-route adjustments beyond `DecodeFinal`. Use that tag in the
worktree command when you want the newest local team-code reference. See the
[comparison report](../../../../../../../../docs/limelight-retirement.md).
