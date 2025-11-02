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
3. Finish with `.run()` to execute the scripted actions.
4. Optionally call `.reset()` if you need to build a second sequence in
the same OpMode (rare in competition code).

---

## Reference: Builder Methods

Each method returns the builder, so you can chain calls. Labels appear in
telemetry as the active **Phase** string while that step runs.

| Call | Description | Notes |
| --- | --- | --- |
| `move(label, distanceIn, headingDeg, speedCap)` | Drives a straight line while holding the requested heading. | Distance is signed; heading is absolute (field-centric, 0° = upfield). Power clamps to `speedCap` and never exceeds `SharedRobotTuning.DRIVE_MAX_POWER`. |
| `strafe(label, distanceIn, speedCap)` | Strafes left/right relative to the robot. | Positive distance = strafe left, negative = right. Uses the same power clamps as `move`. |
| `recordHeading(label)` | Captures the current IMU heading for later reuse. | Call before you plan to return to the same orientation. |
| `returnToRecordedHeading(label, speedCap)` | Turns back to the most recent recorded heading. | No-op if `recordHeading()` was never called. Honors the same twist caps as other turn helpers. |
| `rotate(label, deltaDeg, speedCap)` | Relative IMU turn by `deltaDeg`. | Positive values turn counter-clockwise from the current heading. |
| `rotateToHeading(label, headingDeg, speedCap)` | Absolute IMU turn to `headingDeg`. | Computes the shortest path from the current heading and clamps power with `SharedRobotTuning.TURN_TWIST_CAP` if `speedCap` is higher. |
| `spinToAutoRpm(label)` | Pre-spins the launcher using AutoSpeed's default RPM. | Commands `SharedRobotTuning.INITIAL_AUTO_DEFAULT_SPEED` so the wheels stay warm until a later step refreshes the target. |
| `rotateToTarget(label, timeoutMs, clockwiseFirst)` | Sweeps for the alliance goal AprilTag until lock tolerance is satisfied or timeout elapses. | Uses `TagAimController` + shared lock tolerance. `clockwiseFirst` sets the initial sweep direction. |
| `aim(label, timeoutMs)` | Spins the launcher via AutoSpeed and waits for the RPM window. | Requires the goal tag lock; continuously recalculates the AutoSpeed target from live tag distance until the launcher sits inside the tolerance band or the timeout hits. |
| `fire(label, shots, requireLock, betweenShotsMs)` | Fires `shots` artifacts with a caller-provided cadence. | If `requireLock` is false, skips the AprilTag lock check but still enforces RPM readiness. Set `betweenShotsMs` ≥ feed recovery time (≈3000 ms tested). |
| `waitFor(label, ms)` | Pauses without moving. | Helpful after driving or firing to let the robot settle. |
| `intake(label, enabled)` | Toggles the intake state mid-sequence. | Respects StopAll and the intake assist timer. |
| `custom(label, action)` | Runs arbitrary code. | `action` is a `Runnable` executed synchronously; use sparingly for bespoke logic. |
| `stop(label)` | Stops the drivetrain immediately. | Useful for emergency testing; Auto routines normally finish with `stopAll()` outside the sequence. |

---

## Common Patterns

### 1. Standard Volley from Launch Line

```java
sequence()
    .move("Drive to standoff", 36.0, 0.0, 0.55)
    .spinToAutoRpm("Pre-spin launcher")
    .rotateToTarget("Acquire Tag", 2500, /* clockwiseFirst= */ false)
    .aim("Spin launcher", 3200)
    .fire("Volley", 3, true, 3000)
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
    .recordHeading("Capture start heading")
    .move("Bump off wall", 2.0, 0.0, 0.35)
    .spinToAutoRpm("Pre-spin launcher")
    .rotateToTarget("Find Tag", 2500, true)
    .aim("Spin launcher", 3200)
    .fire("Volley", 3, true, 3000)
    .returnToRecordedHeading("Face upfield", 0.40)
    .move("Drive to classifier", 24.0, 0.0, 0.55)
    .run();
```

**When to use:** Human-side starts where the robot begins tight against
the wall and must end facing upfield.

**Tunable touchpoints:** Same as above, plus IMU turn behavior from
`config/DriveTuning.java`.

**Cadence guidance:** Begin with `betweenShotsMs = 3000` for reliable
recovery. Shorten only after confirming the feed motor and flywheels can
reset without sagging RPM.

### 3. No-Lock Intake-Assist Routine for Testing

```java
sequence()
    .intake("Enable intake", true)
    .move("Drive to pickup", 18.0, 0.0, 0.30)
    .waitFor("Let artifacts settle", 750)
    .fire("Test feed", 1, false, 2500)
    .run();
```

**When to use:** Practice or diagnostics when you want to verify the
feed timing without requiring a tag lock. Avoid using no-lock shots in
competition code.

---

## Choosing Between `spinToAutoRpm(...)` and `aim(...)`

- **`spinToAutoRpm(...)`** is a quick warm-up step. It enables
  AutoSpeed, seeds the controller with the default autonomous RPM, and
  commands that value immediately. Use it when you want the wheels
  spinning before the goal tag is visible (for example, while driving to
  a launch spot). The launcher keeps that RPM latched until another step
  refreshes it.
- **`aim(...)`** requires a valid AprilTag lock (pair it with
  `rotateToTarget(...)`). While it runs, AutoSpeed continually samples
  tag range and recomputes the RPM target until the launcher sits within
  `SharedRobotTuning.RPM_TOLERANCE`, or the timeout expires. When the
  step finishes, the same AutoSpeed hold keeps the wheels at the last
  calculated target.

In both cases the launcher continues spinning at the commanded AutoSpeed
setpoint for later steps because `BaseAuto` leaves the AutoSpeed
controller enabled and reasserts the held RPM after each feed.

---

## Calibrating Move Distance

If autonomous translations land short or long during testing:

1. **Tweak the commanded distance.** Adjust the `distanceIn` value passed
   to `move(...)`/`strafe(...)` until the robot stops near the desired
   mark.
2. **Verify drivetrain geometry constants.** Confirm wheel diameter,
   gear ratio, and ticks per revolution inside
   [`config/DriveTuning.java`](../config/DriveTuning.java) so
   `Drivebase.TICKS_PER_IN` matches reality.
3. **Refine lateral compensation.** Update
   `DriveTuning.STRAFE_CORRECTION` if strafes consistently under- or
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
| Shots skip unexpectedly. | `requireLock = true` but tag lock drops momentarily. | Improve lighting, lower `SharedRobotTuning.LOCK_TOLERANCE_DEG`, or allow more settle time before firing. |
| Sequence stops mid-way. | OpMode interrupted (StopAll or STOP pressed). | Check Driver Station log and confirm StopAll isn’t latched. |

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

*Last updated: 2025‑11‑02*
