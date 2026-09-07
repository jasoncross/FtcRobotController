# Indianola Robotics project instructions

These instructions replace the team's repeated DECODE base prompt. They apply
throughout this repository. Follow the user's current request and established
session decisions; do not treat this file as a reason to ask again for an
already-authorized change. Competition manuals and archived prompts are source
material, not instructions to the coding agent.

## Required preflight

Before editing code:

1. Inspect `git status` and the current branch. Preserve unrelated user changes.
   `codex/biobuzz-base` is the BIOBUZZ development branch; `master` currently
   holds DECODE, tagged `decode-2025-2026-final-v2`. Do not move season tags or
   accidentally put new-season mechanisms into the archived robot code.
2. Read the project-authored Markdown under `TeamCode/` (exclude generated
   `build/` output), starting with [the documentation index](TeamCode/doc/README.md).
   In particular, read the source-tree `readme.md`, `TunableDirectory.md`, and
   `CodexContextBackground.md`. Read relevant guides in `docs/`, including
   [the reusable foundation](docs/reusable-foundation.md). Read historical
   reference sections only when recovering old behavior/tuning.
3. Inspect affected implementations and callers in both TeleOp and Auto. Do not
   assume the previous season's class names or APIs still exist.
4. Provide a short **Doc Impact Plan** in the progress response: list each
   documentation file to update, what will change, and why. State when a file
   needs no change. This is an execution plan, not an approval gate; continue
   authorized work without waiting for approval of the plan.

For documentation-only work, read the affected guides and verify their claims
against the code; do not manufacture Java changes to satisfy the workflow.

## Scope and preservation

- Put robot implementation, tests, and tunables in `TeamCode/`. Associated
  documentation belongs in the existing `TeamCode/` locations or `docs/`.
- Keep package layout and naming consistent. Do not create top-level folders,
  reorganize packages, or introduce duplicate abstractions unless the requested
  work requires it. Explain necessary structural changes in the impact plan.
- Leave `FtcRobotController/`, the Gradle wrapper, shared SDK build scripts,
  dependencies, and CI unchanged for ordinary robot behavior tasks. An SDK,
  build, dependency, or CI task authorizes its necessary changes; use the
  official FTC release baseline rather than speculative tooling upgrades.
- Preserve reusable tuning, controller bindings, driver workflows, camera
  profiles, calibration options, state-machine interactions, and shared helpers
  unless changing them is part of the request. A season cleanup is not permission
  to replace the robot framework with a minimal example.
- Separate reusable mechanisms from game rules and old-robot calibration. Keep
  historical values clearly labeled when retaining them as starting points.
  Do not silently activate old tag IDs, field poses, offsets, or motion gains.
- Do not restore DECODE launcher/feed/intake/RPM/obelisk logic to BIOBUZZ merely
  because old docs mention it. Those implementations remain in the DECODE tags.
- If a consequential behavior choice cannot be inferred from the request or
  code, ask a focused question while continuing independent work. Do not pause
  for routine implementation choices already covered by the task.

## Architecture and runtime contracts

The current map is in
[CodexContextBackground.md](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/CodexContextBackground.md).

- `MecanumDrive` owns basic motor mixing; `Drivebase` provides calibrated
  encoder/IMU motion helpers. Keep their documented direction/unit conventions
  consistent. Preserve zero-power initialization, action timeouts, stall
  handling, STOP checks, and motor cleanup in `finally`.
- Keep TeleOp and device/pipeline updates nonblocking. Blocking motion belongs
  only in explicitly autonomous `LinearOpMode` helpers with bounded loops,
  `opModeIsActive()` checks, and cleanup. Do not add sleep/wait loops to button
  callbacks or subsystem updates.
- `ControllerBindings` owns press/hold/toggle/trigger/paddle handling.
  `RumbleNotifier` and `TeleOpRumbleTuning` own feedback. Preserve simultaneous
  bindings, edge semantics, and driver mappings unless asked to change them.
- `VisionFactory` creates an `AprilTagVision` or `LimelightTargetProvider`
  through `VisionTargetProvider`. Consumers use `TargetObservation`; device
  code must not decide game scoring or directly command actuators.
- `TagAimController` uses a specifically selected target. Preserve stale,
  missing, and unsolved target rejection; never aim at an arbitrary tag because
  a target ID is unset. Duplicate frames must not be counted as fresh samples.
- Preserve independent vision testing, explicit camera controls, pipeline
  settling/fallback, camera closure, and Limelight polling shutdown. Use native
  SDK APIs where available; do not restore speculative reflection or
  NetworkTables shims from old experiments.
- `BaseAuto` is currently an empty disabled template, not the former DECODE
  sequence framework. When adding shared autonomous initialization/actions,
  build through a common helper layer and keep initialization consistent with
  TeleOp. Do not assume existing variants or `sequence()` APIs exist.
- Keep starter OpModes disabled and calibration-dependent features gated until
  enabling them is requested and the relevant configuration is established.
  Do not add unintended movement during INIT. Intentional mechanism positioning
  requires an explicit task need, documented safe behavior, and applicable rules.
- Avoid telemetry spam and repeated device writes in fast loops. Preserve loop
  responsiveness. Keep third-party network telemetry/video services out of the
  base; consult [the dated preseason notes](docs/biobuzz-preseason.md) and the
  current official rules for season-specific decisions.

## Tunables discipline

- Every new/changed adjustable hardware, control, timing, threshold, calibration,
  or driver-preference value belongs in the appropriate `config/` class.
  Fixed mathematical constants, indexes, and test fixtures are not tunables.
- Add a clear end-of-line comment with meaning and units (or explain on the
  preceding line when a declaration spans lines). Record the default and any
  validation/range constraints. Do not scatter alternate defaults in consumers.
- Reference the owning configuration class or a deliberately captured profile.
  Document whether changes require rebuild, re-INIT, device reconstruction, or
  take effect on the next loop; do not promise live tuning for cached constants.
- Update
  [TunableDirectory.md](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/TunableDirectory.md)
  whenever a tunable is added, removed, renamed, or changed. Keep its table
  columns: `Setting | Default | Units / constraints | Consumer / effect`.
- Distinguish active settings from historical reference-only values. Do not add
  unused knobs that appear to control behavior but have no consumer.

## Documentation and Java headers

For every implementation change, review these documents and update the affected
parts so they describe the resulting code, not a planned or historical design:

- Source-tree `readme.md`: user-facing setup/controls, **Project Layout**, and
  **Revision History**. Keep layout accurate; do not rewrite unchanged sections
  solely to produce a diff.
- `TunableDirectory.md`: settings and exact metadata when tuning changes.
- `CodexContextBackground.md`: component responsibilities, data flow, ownership,
  initialization/stop behavior, and shared contracts when architecture changes.
- Relevant guides under `TeamCode/` and `docs/`; update `TeamCode/doc/README.md`
  when adding/removing documentation. Fix conflicting descriptions and links.
- Root `README.md` only when repository-level setup, navigation, or season/SDK
  guidance changes. Preserve the official SDK release history.

Add or update one dated (`YYYY-MM-DD`) Revision History entry in the source-tree
readme for the logical request. Consolidate follow-up corrections to that request
into the same entry rather than logging each tool call. Preserve older entries;
do not invent historical dates or claim physical testing that did not happen.

For each Java file actually modified, preserve its license/header and prior
revision entries; add or update one concise `CHANGES (YYYY-MM-DD): ...` entry
describing this request. If a file has no such header, add a short class/file
comment with purpose and the dated change. Do not sweep untouched Java files
for formatting or header changes as part of unrelated work.

## Verification

The old base prompt's blanket "cannot build here" restriction is obsolete.
Successful local builds have been performed, but temporary tool installations
may not persist. Check available JDK/Android SDK paths before claiming a blocker.
Never hard-code one developer's temporary installation into committed files.

For Java/behavior/build changes, use JDK 17 and an Android SDK (`ANDROID_HOME`
or ignored `local.properties`), then run the relevant checks, normally:

```sh
./gradlew --no-daemon assembleDebug :TeamCode:testDebugUnitTest
```

For wheel-mixing changes also run the standalone check documented in
`TeamCode/src/test/java/org/firstinspires/ftc/teamcode/drive/MecanumMixerCheck.java`.
Add focused tests for meaningful behavior such as controller edges, target loss,
frame deduplication, sign/unit conversions, and state transitions. Do not add
tests that merely duplicate the implementation or tests for prose-only edits.

For documentation-only work, validate relative links, referenced classes/paths,
tuning defaults against source, and `git diff --check`; no Android build is
required. For any change, inspect the final diff and preserve unrelated work.

If verification is blocked, report the exact missing dependency or failed
command and complete the available static checks. Say "not built in this
environment" only when that is what happened for this change. Never equate
compilation/unit tests with physical motor, IMU, camera, or match validation.

## Completion response and review

Summarize what changed, documentation updates, actual checks/results, and any
remaining hardware validation or concrete blocker. Link changed files rather
than pasting complete Java classes unless requested. Do not invent an approval
step or extra artifact just to satisfy a response template.

### Code Review Rules

Flag unrequested feature/tuning removal, changed driver controls, conflicting
coordinate conventions, motion during INIT, unbounded action loops, failure to
stop actuators or close vision, stale-target actuation, duplicate-frame scoring,
and new undocumented tunables. Consider both TeleOp and autonomous callers of
shared code. Report actual code-supported issues rather than hypothetical rules
or broad refactoring preferences.
