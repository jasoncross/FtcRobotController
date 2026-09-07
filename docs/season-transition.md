# DECODE to BIOBUZZ

## Preserved release

The fork's `speed-tweaks` and `master` both ended at `531e546`. Its TeamCode
matched the team's existing `DecodeFinal` tag (`37b191f`) byte-for-byte.
The histories were merged on local `master`, using the complete `DecodeFinal`
tree, and tagged `decode-2025-2026-final` before any preseason changes.
The existing `DecodeFinal` tag was not moved. The merge also preserves the
fork's history, including its historical `TeamCode.zip`.

A subsequent comparison with the team's local repository found a clean February
17 checkout at `69bb375`, one commit after `DecodeFinal`. It adds two Target 9
autonomous modes and adjusts two Human routes. The full newer snapshot is now
preserved as `archive/decode/team-checkout-2026-02-17`; neither existing final
tag was moved. See the [comparison report](limelight-retirement.md) for evidence
and the distinction between the original tagged release and the later checkout.

Those February 17 changes have now been merged into `master` and tagged
`decode-2025-2026-final-v2`. Its full file tree matches `69bb375`, while its merge
history also retains the fork's commits. This is the updated final-season release
to use when the Target 9 modes are needed; the original tags remain unchanged.

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

## Initial cleanup and subsequent restoration

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

Following review, the reusable controller bindings, rumble envelopes, drive
motion helpers and tuning were restored. The webcam/Limelight abstraction,
camera profiles, pipeline selection, and tag-aim tuning are now available as
season-neutral infrastructure. Historical geometry/calibration is labeled and
requires review before use. The [foundation guide](reusable-foundation.md)
describes the current code; the [complete historical tuning reference](decode-tuning-reference.md)
preserves every former configuration file. DECODE mechanisms and field rules
remain excluded from BIOBUZZ.

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

The restored foundation retains the official SDK/toolchain baseline, adding a
JUnit test dependency only in TeamCode. The updated DECODE v2 release also
passed `assembleDebug` with its original SDK 11.0 dependencies.

Run `./gradlew --no-daemon assembleDebug` with JDK 17 and an Android SDK configured
through `ANDROID_HOME` or `local.properties`. GitHub Actions also builds debug
APKs on pushes and pull requests. The pure mecanum math has a standalone Java
check in `TeamCode/src/test/java`; see its header for the command.

Compilation does not validate physical wheel directions, camera operation,
or driving behavior. Those checks require the new robot.
