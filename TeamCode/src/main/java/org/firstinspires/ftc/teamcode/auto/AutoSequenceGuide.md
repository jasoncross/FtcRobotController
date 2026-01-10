# AutoSequence Builder Guide

This guide expands on the `AutoSequence` fluent API that lives inside
[`auto/BaseAuto.java`](./BaseAuto.java). The builder lets you script
autonomous routines as a readable chain of **movement**, **rotation**,
**aim**, **firing**, and **wait** steps without touching the shared drive
and launcher helpers. Every call automatically honors the tunables in
[`config/SharedRobotTuning.java`](../config/SharedRobotTuning.java) and
the cadence/lock safeguards in `BaseAuto`.

---

## When to Use AutoSequence

Use `sequence()` when you need to:

- Add a brand-new autonomous routine.
- Reorder or retune the existing Blue/Red Human or Target autos.
- Build a temporary testing path for calibration.

Because the builder exposes every step declaratively, you can tweak
distances or headings in one place while keeping telemetry and safety
behavior consistent across all autos.

### Lifecycle

1. Call `sequence()` inside your `runSequence()` override.
2. Chain the desired steps in the order they should execute.
3. Use `endgameMove(...)` or `addEndgameStep(...)` for the final retreat actions that should wait until the MAIN phase is over.
4. Finish with `.run()` to execute the scripted actions.

AutoSequence runs MAIN steps until they finish or `BaseAuto.mainPhaseOver()` flips true (30 s total minus the per-OpMode endgame
reserve). ENDGAME steps can run immediately after MAIN completes, and they will also run if MAIN time expires mid-sequence.

### Start pose seeding & TeleOp handoff

- Use `setStartingPose(x, y, headingDeg)` inside your auto class (constructor or field initializer) to seed the fused odometry pose during INIT. This is a BaseAuto helper, **not** a chained AutoSequence step, so it fires once up front rather than inside `sequence()`. BaseAuto aligns the IMU heading offset to that seed so heading does not snap back to IMU zero.
- BaseAuto can optionally re-localize from AprilTags during INIT **only** when `VisionConfig.LimelightFusion.INIT_ALLOW_VISION_SEED` is true; otherwise the explicit start pose remains authoritative until tags are fused later in the run.
- At the end of each Auto, the latest fused pose is written to the shared `odometry/PoseStore`, and TeleOp reads it during INIT (or re-localizes from tags) so field-aware telemetry and Dashboard drawings remain continuous between phases. TeleOp keeps applying tag corrections whenever a goal tag is visible.

---

### Updating the Start Pose Telemetry Line

Every autonomous class extends `BaseAuto` and surfaces its starting
instructions through `startPoseDescription()`. That string feeds the
Driver Station telemetry line labelled **Start Pose** during the INIT
loop. Whenever you adjust the opening location or robot orientation,
update the return value in your auto class so field crews receive the
correct staging reminder (for example, "Start: Blue Human — West of south
firing triangle, FACING NORTH").

Keeping the text synchronized with the actual setup spot prevents crews
from launching a route from the wrong tile after code changes.

---

## Reference: Builder Methods

Each method returns the builder, so you can chain calls. Labels appear in
telemetry as the active **Phase** string while that step runs. The newest
field-aware helpers to know about are:

- `moveToPosition(...)` – field-centric drive to a target odometry coordinate.
- `intakeFieldArtifacts(...)` – alliance/obelisk-aware artifact pickup using the intake offset + odometry tunables.

> **Note:** `setStartingPose(...)` is a BaseAuto INIT helper, not a chained builder call. See the "Start pose seeding" section above for usage.

| Call | Description | Notes |
| --- | --- | --- |
| `rememberHeading(label)` | Captures the current IMU heading for later reuse (non-blocking). | Call before you plan to return to the same orientation; the step runs instantly without pausing surrounding motion. |
| `move(label, distanceIn, headingDeg, twistDeg, speedCap)` | Drives a straight line while steering toward the requested heading change so the robot ends the move at the starting heading plus `twistDeg`. | Distance is signed; `headingDeg` is **relative to the robot’s current facing** (0° = drive straight ahead, 180° = drive straight backwards) and is converted to an absolute heading at runtime so paths stay robot-centric regardless of field orientation. `twistDeg` is relative to the heading at the start of the step (0° to maintain orientation). Power clamps to `speedCap` and never exceeds `SharedRobotTuning.DRIVE_MAX_POWER`; the twist controller shares the same cap while blending rotation during the translation. |
| `endgameMove(label, distanceIn, headingDeg, twistDeg, speedCap)` | Schedules a move in the ENDGAME phase so it waits until the MAIN phase is over (per the endgame reserve). | Use for final retreat moves that must start near the end of autonomous without interrupting normal MAIN steps. |
| `moveToPosition(label, targetX, targetY, headingDeg, speedCap)` | Field-centric drive to a specific odometry coordinate using fused odometry (drive encoders + IMU + AprilTags) so the robot center arrives at `targetX, targetY` and finishes at `headingDeg`. | Uses `BaseAuto.moveToPosition(...)` helper; clamps speed to `SharedRobotTuning.DRIVE_MAX_POWER` and reuses turn tolerances from drive helpers. |
| `rotate(label, deltaDeg, speedCap)` | Relative IMU turn by `deltaDeg`. | Positive values turn counter-clockwise from the current heading. |
| `rotateToHeading(label, headingDeg, speedCap)` | Absolute IMU turn to `headingDeg`. | Computes the shortest path from the current heading and clamps power with `SharedRobotTuning.TURN_TWIST_CAP` if `speedCap` is higher. |
| `spinToAutoRpmDefault(label)` | Pre-spins the launcher using AutoSpeed's default RPM (non-blocking). | Commands `SharedRobotTuning.INITIAL_AUTO_DEFAULT_SPEED` so the wheels stay warm until a later step refreshes the target without stopping drive motion. |
| `rotateToTarget(label, direction, turnSpeedFraction, primarySweepDeg, oppositeSweepDeg, timeoutMs)`<br/>`rotateToTarget(label, turnSpeedFraction, primarySweepDeg, oppositeSweepDeg, timeoutMs)`<br/>`rotateToTarget(label, direction, turnSpeedFraction, primarySweepDeg, timeoutMs)`<br/>`rotateToTarget(label, turnSpeedFraction, primarySweepDeg, timeoutMs)` | Sweeps for the alliance goal AprilTag using repeatable angular passes until lock tolerance is satisfied (or the timeout elapses). | Pass `ScanDirection.CW/CCW` (or omit to default clockwise) to set the opening sweep. `turnSpeedFraction` scales the shared twist cap (0–1). `primarySweepDeg` sets how far to travel in the opening direction. `oppositeSweepDeg` governs the counter sweep: positive values cross through zero into the opposite side by that magnitude, negative values stop short of zero by that magnitude before reversing, zero returns to center before heading back out, and omitting the argument holds at the primary sweep limit with no counter pass. Provide `timeoutMs` per call (autos currently pass **10 000 ms** = **10 s**) to abort the scan if no tag lock is found. |
| `visionMode(label, mode)` | Swaps the AprilTag vision profile mid-sequence. | Calls `VisionAprilTag.applyProfile(...)`, reapplies `VisionTuning.RANGE_SCALE`, and logs the active resolution so you can flip between `P480` and `P720` before aim steps. When `VisionConfig.VISION_SOURCE` is `LIMELIGHT`, this step is ignored because Limelight does not expose these webcam profiles. |
| `readyToLaunch(label, timeoutMs)`<br/>`readyToLaunch(label, timeoutMs, launchDistanceIn)` | Spins the launcher via AutoSpeed and waits for the RPM window + settle timer. | Requires the goal tag lock; continuously recalculates the AutoSpeed target from live tag distance until the launcher stays inside the tolerance band for `SharedRobotTuning.RPM_READY_SETTLE_MS` or the timeout hits. When the optional `launchDistanceIn` is provided, AutoSpeed uses that distance to seed RPM until a real tag distance arrives, then overrides it with the live lock. |
| `fire(label, shots, requireTargetLock, requireLauncherAtSpeed, betweenShotsMs)` | Fires `shots` artifacts with a caller-provided cadence. | If `requireTargetLock` is false, skips the AprilTag lock check. If `requireLauncherAtSpeed` is true, waits for the RPM window before each shot. Set `betweenShotsMs` ≥ feed recovery time (≈3000 ms tested). |
| `fireContinuous(label, timeMs, requireTargetLock, requireLauncherAtSpeed)` | Holds the FeedStop open and streams feed power until `timeMs` elapse. | Keeps AutoSpeed enabled so launcher RPM stays latched; when `requireTargetLock` is true the step is skipped if the prior aim step never achieved lock, and the builder now remembers a successful `readyToLaunch(...)` lock so continuous fire will proceed instead of being dropped. `requireLauncherAtSpeed` gates only the initial launch before the stream begins. |
| `waitFor(label, ms)` | Pauses without moving. | Helpful after driving or firing to let the robot settle. |
| `eject(label)` | Runs the TeleOp eject routine mid-auto. | Temporarily overrides AutoSpeed, spins to `TeleOpEjectTuning.RPM`, feeds once with intake assist, then restores the previous RPM/AutoSpeed state. |
| `intake(label, enabled)` | Toggles the floor intake on or off. | Adds telemetry showing the requested state before calling `intake.set(enabled)`. Useful for pickup experiments without writing custom steps. |
| `intakeFieldArtifacts(label, reverseSpeed)` | Alliance-aware intake run that aligns the intake mouth to the correct artifact row based on Vision/Obelisk ID (GPP/PGP/PPG) and odometry tunables, then backs into the row while running the intake at `reverseSpeed`. | Uses `BaseAuto.intakeFieldArtifacts(...)` to compute the target pose from odometry + intake offset; honors existing intake state machine without blocking beyond allowed BaseAuto waits. |
| `stop(label)` | Stops drive, launcher, feed, intake, and AutoSpeed. | Calls `stopAll()` so you can insert a hard safety stop mid-sequence. |
| `returnToStoredHeading(label, speedCap)` | Turns back to the most recent stored heading. | No-op if `rememberHeading()` was never called. Honors the same twist caps as other turn helpers. |
| `setStartingPose(x, y, headingDeg)` *(INIT helper)* | Seeds the fused odometry pose during INIT before the OpMode starts. | Call inside your auto class (outside the `sequence()` chain) to declare the expected robot-center pose on the field; BaseAuto and TeleOp also use this hook when re-localizing from goal tags so Auto→TeleOp pose handoff stays continuous. |
| `custom(action)` / `custom(label, action)` | Runs arbitrary code. | `action` is an `AutoStep` executed synchronously; use for bespoke logic such as toggling vision pipelines or adjusting subsystem states. |
| `addEndgameStep(action)` | Adds a custom ENDGAME step. | Runs after MAIN time expires; useful for one-off retreat steps without building a full helper. |

> **Scan sweep primer:** `rotateToTarget(...)` multiplies `SharedRobotTuning.TURN_TWIST_CAP` by the provided `turnSpeedFraction` to compute the twist command while scanning. When the goal tag is not visible, the helper drives the robot to the requested `primarySweepDeg`, then follows the counter-sweep rule you provide before repeating (unless you omit the counter sweep entirely). Passing a **positive** `oppositeSweepDeg` pushes through zero into the other direction (for example, `.rotateToTarget("Scan", ScanDirection.CCW, 0.25, 90, 30)` visits +90°, returns to 0°, checks -30°, and comes back to center). Passing a **negative** value stops short of zero by that magnitude so the scan bounces on the same side (for example, `.rotateToTarget("Scan", ScanDirection.CCW, 0.25, 180, -90)` swings 180° counter-clockwise, returns clockwise to +90°—still on the counter-clockwise side of center—and heads back toward 180°). Passing **zero** returns to center before re-running the primary sweep, and omitting `oppositeSweepDeg` holds at the primary sweep limit with no return leg. Choose larger sweep angles when you need to search a wider arc; tweak `TURN_TWIST_CAP` to globally change the underlying twist cap.

---

## TEST: Drive Distance Tuner

The `TEST: Drive Distance Tuner` OpMode (`auto/TEST_DriveDistanceTuner.java`) is a dashboard-driven
AutoSequence harness that runs exactly one movement per repetition. It exposes distance, heading,
twist, and speed caps so you can tune encoder translation and twist behavior without editing code.

### Test modes
- **FORWARD** – `move(distance, heading 0°, twist 0°)`
- **BACKWARD** – `move(distance, heading 180°, twist 0°)`
- **STRAFE_LEFT** – `move(distance, heading −90°, twist 0°)`
- **STRAFE_RIGHT** – `move(distance, heading 90°, twist 0°)`
- **DIAGONAL_45** – `move(distance, heading 45°, twist 0°)`
- **DIAGONAL_NEG_45** – `move(distance, heading −45°, twist 0°)`
- **ROTATE_IN_PLACE** – `rotate(ROTATE_DEG)`
- **MOVE_WITH_TWIST** – `move(distance, heading 0°, twist = TWIST_DEG)`
- **CUSTOM** – `move(distance, heading = HEADING_DEG, twist = TWIST_DEG)`

### Recommended usage
- **REPS:** Start with `1–3` reps to keep tuning quick and prevent overheating.
- **Distances:** Try **24 / 48 / 72 in** while verifying odometry drift.
- **Speed cap:** Begin at `SPEED_CAP = 1.0` and reduce only if traction slips.

---

## Common Patterns

### 1. Standard Volley from Launch Line

```java
sequence()
    .move("Drive to standoff", 36.0, 0.0, 0.0, 0.55)
    .spinToAutoRpmDefault("Pre-spin launcher")
    .rotateToTarget("Acquire Tag", ScanDirection.CCW, 0.25, 90, 30, 10000)
    .readyToLaunch("Ready launcher", 3200)
    .fire("Volley", 3, true, true, 3000)
    .waitFor("Stabilize", 500)
    .run();
```

**When to use:** Starting on the depot launch line with a clear path to
the goal tag.

**Tunable touchpoints:**
- `SharedRobotTuning.DRIVE_MAX_POWER` caps translation speed.
- `SharedRobotTuning.LOCK_TOLERANCE_DEG` controls when the tag lock is
  considered good enough to shoot.
- `SharedRobotTuning.INITIAL_AUTO_DEFAULT_SPEED` seeds the pre-spin RPM
  before the first tag lock.

### 2. Human-Side Bump, Fire, and Drive Upfield

```java
sequence()
    .rememberHeading("Capture start heading")
    .move("Bump off wall", 2.0, 0.0, 0.0, 0.35)
    .spinToAutoRpmDefault("Pre-spin launcher")
    .rotateToTarget("Find Tag", ScanDirection.CW, 0.25, 90, 30, 10000)
    .readyToLaunch("Ready launcher", 3200)
    .fire("Volley", 3, true, true, 3000)
    .returnToStoredHeading("Face upfield", 0.40)
    .move("Drive to classifier", 24.0, 0.0, 0.0, 0.55)
    .run();
```

**When to use:** Human-side starts where the robot begins tight against
the wall and must end facing upfield.

**Tunable touchpoints:** Same as above, plus IMU turn behavior from
`config/DriveTuning.java`.

**Cadence guidance:** Begin with `betweenShotsMs = 3000` for reliable
recovery. Shorten only after confirming the feed motor and flywheels can
reset without sagging RPM.

### 3. Endgame Retreat with a Custom Step

```java
sequence()
    .rotateToTarget("Find Tag", ScanDirection.CW, 0.25, 90, 30, 10000)
    .readyToLaunch("Ready launcher", 3200)
    .fire("Volley", 3, true, true, 3000)
    .addEndgameStep(() -> drive.move(8.0, 0.0, 0.6))
    .run();
```

**When to use:** When you need a quick, one-off ENDGAME action without a
dedicated helper (for example, a short retreat move).

### 4. Clearing a Jam Mid-Route

```java
sequence()
    .fire("Volley", 3, true, true, 3000)
    .waitFor("Check for jam", 500)
    .eject("Clear feeder")
    .waitFor("Let debris exit", 400)
    .run();
```

**When to use:** Practice paths where you intentionally inject a jam or
want a one-button recovery after detecting a misfire. The eject step
mirrors the TeleOp B-button behavior: it temporarily overrides AutoSpeed,
feeds once with intake assist, and then restores the prior RPM.

### 5. Custom Intake-Assist Routine for Testing

```java
sequence()
    .custom("Enable intake", () -> intake.set(true))
    .move("Drive to pickup", 18.0, 0.0, 0.0, 0.30)
    .waitFor("Let artifacts settle", 750)
    .fire("Test feed", 1, false, true, 2500)
    .custom("Disable intake", () -> intake.set(false))
    .run();
```

**When to use:** Practice or diagnostics when you want to verify the
feed timing without requiring a tag lock. Avoid using no-lock shots in
competition code.

**Timeout behavior:** Steps with timeouts (for example, `rotateToTarget(...)`
or `readyToLaunch(...)`) now advance as soon as their goals are satisfied;
the timeout acts only as a fallback when locks/at-speed gates never clear.
This keeps sequences moving instead of idling until the full timeout expires
after a successful aim or launcher prep.

---

## Additional `.custom(...)` Ideas

The `.custom(...)` hook runs inside your auto class, so you can access the
same subsystems that `BaseAuto` exposes. Common use cases include:

- **Vision profile swaps:** Use `.visionMode("Switch to 720p", VisionTuning.Mode.P720)` when you want to change pipelines mid-run without writing a custom block.
- **Sensor logging:** Push a one-off telemetry line (for example, range
  sensor distance) before taking an action.
- **Conditional logic:** Check a sensor and branch by queuing additional
  steps—build the guard in the custom block and append more `sequence()`
  calls when needed.

Keep custom steps short and deterministic; long loops belong in dedicated
builder methods so telemetry remains responsive.

---

## Choosing Between `spinToAutoRpmDefault(...)` and `readyToLaunch(...)`

- **`spinToAutoRpmDefault(...)`** is a quick warm-up step. It enables
  AutoSpeed, seeds the controller with the default autonomous RPM, and
  commands that value immediately. Use it when you want the wheels
  spinning before the goal tag is visible (for example, while driving to
  a launch spot). The launcher keeps that RPM latched until another step
  refreshes it.
- **`readyToLaunch(...)`** requires a valid AprilTag lock (pair it with
  `rotateToTarget(...)`). While it runs, AutoSpeed continually samples
  tag range and recomputes the RPM target until the launcher stays
  within `SharedRobotTuning.RPM_TOLERANCE` for the
  `SharedRobotTuning.RPM_READY_SETTLE_MS` window, or the timeout expires.
  The optional `launchDistanceIn` overload uses that distance to seed
  AutoSpeed when no lock is available yet, then overrides it as soon as
  a live tag distance is seen. When the step finishes, the same AutoSpeed
  hold keeps the wheels at the last calculated target.

In both cases the launcher continues spinning at the commanded AutoSpeed
setpoint for later steps because `BaseAuto` leaves the AutoSpeed
controller enabled and reasserts the held RPM after each feed.

---

## Calibrating Move Distance

If autonomous translations land short or long during testing:

1. **Tweak the commanded distance.** Adjust the `distanceIn` value passed
   to `move(...)` (and its heading argument when strafing) until the robot
   stops near the desired mark.
2. **Verify drivetrain geometry constants.** Confirm wheel diameter,
   gear ratio, and ticks per revolution inside
   [`config/DriveTuning.java`](../config/DriveTuning.java) so
   `Drivebase.TICKS_PER_IN` matches reality.
3. **Refine lateral compensation.** Update
   `DriveTuning.STRAFE_CORRECTION` if sideways moves (heading ±90°) under- or
   overshoot; the value scales the X component inside
   [`Drivebase.move(...)`](../drive/Drivebase.java).

Test after each change and capture field notes so you can revert if a
new constant overshoots.

---

## Troubleshooting Tips

| Symptom | Likely Cause | Fix |
| --- | --- | --- |
| Robot drives too fast or slow. | `speedCap` exceeds or undershoots the shared drive cap. | Adjust the `speedCap` argument **and/or** update `SharedRobotTuning.DRIVE_MAX_POWER`. |
| Aim step times out. | Goal AprilTag not visible or heading far off. | Increase the timeout, adjust starting pose, or verify the tag ID in the pit. |
| Shots skip unexpectedly. | `requireTargetLock = true` but tag lock drops momentarily. | Improve lighting, lower `SharedRobotTuning.LOCK_TOLERANCE_DEG`, or allow more settle time before firing. |
| Sequence stops mid-way. | OpMode interrupted (StopAll or STOP pressed). | Check Driver Station log and confirm StopAll isn’t latched. |

---

## Creating a New Auto Mode Class

1. **Copy an existing template.** Duplicate the closest match (for
   example, `Auto_Blue_Target`) inside
   [`auto/`](./) and rename the class + file.
2. **Update the `@Autonomous` annotation.** Set a unique `name`, the
   correct `group`, and an optional `preselectTeleOp` so the Driver
   Station shows the right pairing.
3. **Override the required hooks.** Implement `alliance()` and
   `startPoseDescription()` to match the new route. Adjust the start pose
   string immediately so telemetry reflects the new staging point.
4. **Decide on your opening sweep.** Pass `ScanDirection.CW/CCW` (or omit
   the direction to default clockwise) plus the speed and sweep angles you
   want into `rotateToTarget(...)`. Larger angles cover more field before
   repeating; higher speed fractions reach those limits faster.
5. **Build the sequence.** Inside `runSequence()` call `sequence()` and
   chain the desired builder steps. Use `.custom(...)` for any bespoke
   logic that does not warrant a dedicated helper yet.
6. **Test on the practice field.** Confirm telemetry shows the updated
   Start Pose, the path respects alliance geometry, and AutoSpeed locks
   before firing.

Follow the checklist below to keep the documentation in sync once the
code lands.

---

## Updating Documentation

Whenever you add or rename an auto class that uses `AutoSequence`,
update:

- [`readme.md`](../readme.md) → **Project Layout** tree and the relevant
autonomous routine section.
- [`TunableDirectory.md`](../TunableDirectory.md) if new steps introduce
fresh tunables under `config/`.

Keeping docs aligned with the code prevents regressions during driveteam
handoffs and mentor reviews.

---

*Last updated: 2026‑01‑09*
