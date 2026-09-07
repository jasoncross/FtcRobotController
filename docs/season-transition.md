# DECODE to BIOBUZZ

## Preserved release

The fork's `speed-tweaks` and `master` both ended at `531e546`. Its TeamCode
matched the team's existing `DecodeFinal` tag (`37b191f`) byte-for-byte.
The histories were merged on local `master`, using the complete `DecodeFinal`
tree, and tagged `decode-2025-2026-final` before any preseason changes.
The existing `DecodeFinal` tag was not moved. The merge also preserves the
fork's history, including its historical `TeamCode.zip`.

The `codex/biobuzz-base` branch then merged team `master` (`5e8e046`), which
already included the official SDK update and removal of most DECODE code.

## SDK baseline

The latest official release verified on September 7, 2026 is
[v11.2.1](https://github.com/FIRST-Tech-Challenge/FtcRobotController/releases/tag/v11.2.1),
commit `26cd1fdd2a3c4b26173d9ff33a3279c27d1c7ad1`.
All FTC Maven dependencies use 11.2.1. Build scripts and wrapper settings use
the official release's Android Gradle Plugin 8.13.2 and Gradle 9.1.0, replacing
the team's later tooling customizations. Use JDK 17 and Android Studio Narwhal 3
Feature Drop or later. FIRST documents Driver Station 11.2 as compatible with
this tooling-only patch release.

This is a BIOBUZZ preparation branch on the current published SDK, not a claim
that the SDK includes BIOBUZZ field data. Check FIRST's releases again when
the season's SDK is published. SDK examples and historical release notes remain
intact, including their references to older games.

## Team code changes

- Removed DECODE launchers, intake/feed sequencing, RPM control, auto-aim,
  autonomous paths, field drawing, localization fusion, tag IDs, old tuning,
  controller bindings, and season-specific development prompts.
- Removed the unused Limelight compatibility helper and FTC Dashboard
  dependency. The current foundation needs neither.
- Replaced the upstream StarterBot hardware assumptions with configurable
  mecanum drive and optional webcam AprilTag vision.
- Added disabled drive, vision-test, and autonomous starter OpModes.
- Kept robot-specific motor names/directions and camera choices in RobotConfig;
  no DECODE encoder scales, IMU orientation, camera offsets, or motion gains
  were reused.

See the [TeamCode setup guide](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/readme.md)
before enabling any starter OpMode.

## Preseason manual and branch housekeeping

The supplied BIOBUZZ Pre-Season V0 manual adds relevant control-system,
vision, network-streaming, and actuator constraints. See
[preseason notes](biobuzz-preseason.md) for rule/page references and details
that remain pending until kickoff. No game-specific timing or field data has
been inferred from this preliminary manual.

See the [branch cleanup assessment](branch-cleanup.md) for merged branches that
can be retired and unmerged work/open PRs that should be preserved first.

## Validation

On September 7, 2026, `assembleDebug` passed using a temporary JDK 17 and
Android SDK. The mecanum checks passed, including 9,261 combined stick inputs.
The final DECODE tag's tree was verified identical to `DecodeFinal`, and the
SDK controller, wrapper, and build files match v11.2.1 apart from trailing
whitespace. All three starter OpModes were verified disabled.

Run `./gradlew --no-daemon assembleDebug` with JDK 17 and an Android SDK configured
through `ANDROID_HOME` or `local.properties`. GitHub Actions also builds debug
APKs on pushes and pull requests. The pure mecanum math has a standalone Java
check in `TeamCode/src/test/java`; see its header for the command.

Compilation does not validate physical wheel directions, camera operation,
or driving behavior. Those checks require the new robot.
