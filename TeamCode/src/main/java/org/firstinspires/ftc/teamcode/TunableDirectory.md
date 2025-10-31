# TeamCode Tunable Directory

This directory clusters every adjustable value in `TeamCode` by what the driver station team actually tunes—launcher speed, drivetrain motion, intake flow, vision alignment, etc.—instead of by source file. Each table calls out where a number lives, whether it affects **TeleOp**, **Autonomous**, or **Both**, how the robot behaves when it changes, and which copy to edit when multiple values interact.

> **How to read the tables**
>
> * **Parameter** — Name of the constant, field, or call that you edit in code.
> * **Set in** — File and class where the authoritative value is defined.
> * **Impacts** — Whether TeleOp, Autonomous, or Both modes feel the change.
> * **What it controls** — Operational context for the tunable.
> * **Tune here vs. elsewhere** — Override relationships so you know which location “wins.”
> * **Sample adjustments** — Common tweaks and why you might choose them.

## Launcher speed & flywheel control

| Parameter | Set in | Impacts | What it controls | Tune here vs. elsewhere | Sample adjustments |
| --- | --- | --- | --- | --- | --- |
| `AutoRpmConfig.NEAR_DIST_IN` / `NEAR_RPM` / `FAR_DIST_IN` / `FAR_RPM` | `config/AutoRpmConfig.java` | Both | Anchor points for the AutoSpeed distance→RPM curve (defaults now 65.4 in → 4550 RPM and 114 in → 5000 RPM). | This file overwrites `LauncherAutoSpeedController` defaults every time `AutoRpmConfig.apply(...)` runs (TeleOp init + Auto helpers). Edit here unless you need a one-off experiment in a fork. | Shift to `70 in → 4600 RPM` if closer volleys float; `118 in → 5050 RPM` for deeper standoffs. |
| `AutoRpmConfig.SMOOTH_ALPHA` | `config/AutoRpmConfig.java` | Both | Exponential smoothing applied to RPM updates. | Overrides the controller’s internal `smoothingAlpha`. Change here to keep Auto and TeleOp aligned. | `0.05` for snappier response, `0.25` to calm oscillations. |
| `AutoRpmConfig.DEFAULT_NO_TAG_RPM` | `config/AutoRpmConfig.java` | Both | RPM held when AutoSpeed is active but no AprilTag distance is available yet. | Seeds the launcher before the first lock; once vision has produced an RPM, AutoSpeed now holds that last computed value through tag dropouts. Update alongside the anchor points to match your preferred standby shot. | Raise to `4600 RPM` if your near shot prefers a hotter standby; drop toward `4300 RPM` to ease battery load during vision dropouts. |
| `LauncherAutoSpeedController.nearDistanceIn` / `nearSpeedRpm` / `farDistanceIn` / `farSpeedRpm` | `control/LauncherAutoSpeedController.java` | Both | Backup defaults for the AutoSpeed curve. | Only take effect if `AutoRpmConfig.apply(...)` is skipped. Tune via `AutoRpmConfig` unless you are intentionally bypassing it. | Use curved anchors for lab testing before copying into `AutoRpmConfig`. |
| `LauncherAutoSpeedController.smoothingAlpha` | `control/LauncherAutoSpeedController.java` | Both | Controller-local smoothing constant. | Immediately replaced by `AutoRpmConfig.SMOOTH_ALPHA`; keep equal so local tests mirror real play. | Match `AutoRpmConfig` while experimenting. |
| `SharedRobotTuning.INITIAL_AUTO_DEFAULT_SPEED` | `config/SharedRobotTuning.java` | Both (seed) | Starting RPM before the first AprilTag lock. | Central value feeds Auto helpers and `assist/AutoAimSpeed`; TeleOp copies it via `TeleOpDriverDefaults.INITIAL_AUTO_DEFAULT_SPEED`. Change it here unless you need a TeleOp-only divergence. | `2800 RPM` for long fields; `2200 RPM` for short volleys. |
| `TeleOpDriverDefaults.INITIAL_AUTO_DEFAULT_SPEED` | `config/TeleOpDriverDefaults.java` | TeleOp | Local override of the seed RPM. | Overrides TeleOp only; Autonomous and `AutoAimSpeed` stay on the shared value. Only edit when drivers want a different opening RPM than Auto. | `2400 RPM` if TeleOp wants faster spin-up than Auto. |
| `LauncherTuning.RPM_MIN` / `RPM_MAX` | `config/LauncherTuning.java` | Both | Software clamp on any RPM request. | Applies after all other tunables. Lowering `RPM_MAX` here limits both AutoSpeed and manual RPM. | Drop max to `5500` if motors saturate; raise min to `200` to keep wheels spinning. |
| `LauncherTuning.PID_P` / `PID_I` / `PID_D` / `PID_F` | `config/LauncherTuning.java` | Both | Velocity loop gains sent to the REV Hub. | Shared across all modes; tune here so Auto and TeleOp remain stable together. | Increase `P` to `12` for heavier wheels; add `D≈0.5` when overshooting. |
| `LauncherTuning.AT_SPEED_TOLERANCE_RPM` | `config/LauncherTuning.java` | Both | Launcher’s own “ready” window. | Used when no custom tolerance is provided; coordinate with `SharedRobotTuning.RPM_TOLERANCE`. | Tighten to `60 RPM` for accuracy; loosen to `120 RPM` if jittery. |
| `LauncherTuning.MANUAL_RPM_STEP` | `config/LauncherTuning.java` | TeleOp | D-pad step applied to manual RPM adjustments when AutoSpeed is OFF **and Manual Lock is engaged**. | Matches RPM Test mode by default; reduce for finer nudges or raise for quicker swings. | `25 RPM` for precise tweaks; `75 RPM` for rapid tuning. |
| `TeleOpDriverDefaults.RPM_BOTTOM` / `RPM_TOP` | `config/TeleOpDriverDefaults.java` | TeleOp | Manual-mode RPM slider limits. | Independent of the launcher clamp; ensure `RPM_TOP ≤ LauncherTuning.RPM_MAX`. | Set bottom to `500 RPM` to keep wheels warm; lower top to `5500 RPM` if browning out. |
| `SharedRobotTuning.RPM_TOLERANCE` | `config/SharedRobotTuning.java` | Both | Global readiness window for AutoSpeed. | Overrides `BaseAuto.rpmTol()` and seeds `assist/AutoAimSpeed.rpmTolerance`; TeleOp follows unless it overrides locally. Tune here for shared behavior. | `35 RPM` for precise volleys; `75 RPM` if batteries sag. |
| `assist/AutoAimSpeed.rpmTolerance` | `assist/AutoAimSpeed.java` | Both | AutoAim-specific readiness window. | Defaults to `SharedRobotTuning.RPM_TOLERANCE`; overriding here affects AutoAim only, not the rest of AutoSpeed usage. | Narrow to `30 RPM` when AutoAim needs stricter gating. |

## Shot cadence, feed, and eject

| Parameter | Set in | Impacts | What it controls | Tune here vs. elsewhere | Sample adjustments |
| --- | --- | --- | --- | --- | --- |
| `SharedRobotTuning.SHOT_BETWEEN_MS` | `config/SharedRobotTuning.java` | Auto | Minimum delay between autonomous shots. | Replaces `BaseAuto.DEF_BETWEEN_MS`; adjust here for global cadence changes. | `2500 ms` for faster volleys once feed is proven. |
| `FeedTuning.INTAKE_ASSIST_MS` | `config/FeedTuning.java` | Both | How long intake runs after a feed when previously off. | Central value now lives alongside feed cadence so Auto and TeleOp share one number; TeleOp copies it via `TeleOpDriverDefaults.INTAKE_ASSIST_MS`. | `350 ms` for sticky rings; `200 ms` when intake clears fast. |
| `TeleOpDriverDefaults.INTAKE_ASSIST_MS` | `config/TeleOpDriverDefaults.java` | TeleOp | TeleOp-only override of intake assist. | Mirrors `FeedTuning.INTAKE_ASSIST_MS`; override here only when drivers want a different assist duration than Auto. | Extend to `320 ms` if TeleOp wants extra assurance. |
| `FeedTuning.FIRE_POWER` | `config/FeedTuning.java` | Both | Motor power used to push a ring. | Shared by Auto `fireN` and TeleOp feed buttons. | Bump to `1.0` when rings stick; lower to `0.75` to reduce jams. |
| `FeedTuning.FIRE_TIME_MS` | `config/FeedTuning.java` | Both | Time the feed motor runs per shot. | Single source for Auto + TeleOp. | `450 ms` after tightening timing; `650 ms` if rings hesitate. |
| `FeedTuning.MIN_CYCLE_MS` | `config/FeedTuning.java` | Both | Cooldown between shots. | Keep consistent so Auto and TeleOp pacing match. | `200 ms` when hardware tolerates quick cycles. |
| `FeedTuning.IDLE_HOLD_POWER` | `config/FeedTuning.java` | Both | Counter-rotation power while the feed is idle. | `Feed` enables this only after START via `setIdleHoldActive(true)` so INIT stays still. | Defaults to `-0.5` to hold artifacts firmly; back off toward `-0.3` if gears chatter. |
| `TeleOpEjectTuning.RPM` | `config/TeleOpEjectTuning.java` | TeleOp | Launcher RPM during eject routine. | TeleOp-only; Auto never calls eject. Tune here without affecting Auto. | `400 RPM` for gentle clears; `800 RPM` for stubborn jams. |
| `TeleOpEjectTuning.TIME_MS` | `config/TeleOpEjectTuning.java` | TeleOp | Duration of eject routine. | TeleOp only. | `600 ms` for quick clear, `1400 ms` for heavy debris. |

## AutoAim, targeting, and AprilTag alignment

| Parameter | Set in | Impacts | What it controls | Tune here vs. elsewhere | Sample adjustments |
| --- | --- | --- | --- | --- | --- |
| `SharedRobotTuning.TURN_TWIST_CAP` | `config/SharedRobotTuning.java` | Both | Maximum twist power for automatic aiming/turns. | Overrides `BaseAuto.turnTwistCap()` and seeds `AutoAimTuning.MAX_TWIST`. Tune centrally for consistent feel. | Raise to `0.45` for aggressive steering; drop to `0.25` if overshooting. |
| `AutoAimTuning.MAX_TWIST` | `config/AutoAimTuning.java` | Both | AutoAim-specific twist clamp. | Defaults to the shared cap; override here only if AutoAim needs a different cap than Auto turn helpers. | Increase slightly above shared cap for faster driver assists. |
| `AutoAimTuning.AUTO_AIM_SPEED_SCALE` | `config/AutoAimTuning.java` | TeleOp | Translation throttle applied while AutoAim is enabled. | TeleOp reads this directly; keep between 0–1 so drivers retain manageable movement during aim assist. | `0.15` for precision lining up, `0.40` if drivers want faster approaches. |
| `SharedRobotTuning.LOCK_TOLERANCE_DEG` | `config/SharedRobotTuning.java` | Auto | Acceptable AprilTag bearing error before firing. | Overrides `BaseAuto.lockTolDeg()` (replacing `DEF_LOCK_TOL_DEG`). Tune here to keep all auto routines consistent. | `0.5°` for precise shots; `1.5°` if heading drifts. |
| `DriveTuning.TURN_TOLERANCE_DEG` | `config/DriveTuning.java` | Both | IMU turn completion tolerance. | Works with `LOCK_TOLERANCE_DEG`; keep both aligned. TeleOp braking also uses this. | Tighten to `0.5°` when Auto must be exact. |
| `TagAimTuning.CLAMP_ABS` | `config/TagAimTuning.java` | Both | Twist limits inside the AprilTag controller. | Ensure the clamp (default ±`0.6`) is ≥ `AutoAimTuning.MAX_TWIST` so AutoAim isn’t double-limited. | Expand to `±0.8` for faster centering. |
| `TagAimTuning.KP` / `KD` | `config/TagAimTuning.java` | Both | PD gains for tag-based twist corrections. | Shared by AutoAim and BaseAuto; tune here for consistent tracking. | `0.025/0.004` for quicker reaction; dial back `kD` if noisy. |
| `SharedRobotTuning.RPM_TOLERANCE` (see above) | `config/SharedRobotTuning.java` | Both | Launch readiness gating for AutoAim + Auto. | Shared reference for all aim subsystems; coordinate with `LauncherTuning.AT_SPEED_TOLERANCE_RPM`. | As above. |
| `AutoAimTuning.INITIAL_AUTO_DEFAULT_SPEED` | `config/AutoAimTuning.java` | Both | AutoAim’s seed RPM before first tag fix. | Mirrors `SharedRobotTuning.INITIAL_AUTO_DEFAULT_SPEED`; override here if AutoAim needs a unique seed. | Match TeleOp override when diverging from shared value. |

## Vision & range calibration

| Parameter | Set in | Impacts | What it controls | Tune here vs. elsewhere | Sample adjustments |
| --- | --- | --- | --- | --- | --- |
| `VisionTuning.RANGE_SCALE` | `config/VisionTuning.java` | Both (shared instance) | Multiplier that converts camera-estimated distance to real inches. | TeleOp initializes the shared `VisionAprilTag` instance with this value, overriding the class default of `1.0`. Updating the scale affects any mode using that instance (Auto, AutoAim). | Calibrate by `trueDistance / measuredDistance`; typically near `1.0`. |
| `VisionTuning.P480_*` constants (via `VisionTuning.forMode(Mode.P480)`) | `config/VisionTuning.java` | Both | Logitech C270 **P480** performance profile: 640×480 @ 30 FPS, decimation, frame gate, min margin, manual exposure/gain/WB lock, and Brown–Conrady calibration. | Update this block when tuning the Control Hub performance stream—TeleOp + Auto load it through `vision.applyProfile(VisionTuning.Mode.P480)` at startup. | Raise exposure/gain for dim venues or tweak decimation/processEveryN for lighter CPU loads. |
| `VisionTuning.P720_*` constants (via `VisionTuning.forMode(Mode.P720)`) | `config/VisionTuning.java` | Both | Logitech C270 **P720** sighting profile: 1280×720 @ 20 FPS with tuned decimation, frame skipping, min margin, and calibrated intrinsics/distortion. | Edit these numbers after capturing your own 720p calibration export; TeleOp’s Gamepad 2 D-pad right applies this profile live. | Reduce `PROCESS_EVERY_N` to `1` if Control Hub CPU headroom allows full-frame processing. |
| `VisionTuning.DEFAULT_PROFILE` | `config/VisionTuning.java` | Both | Which profile loads at startup. | Change when you want TeleOp/Auto to boot into a different stream without editing code. | Set to `PROFILE_720` (or call `forMode(Mode.P720)`) for sighting-focused scrimmages. |
| `VisionTuning.DEFAULT_LIVE_VIEW_ENABLED` | `config/VisionTuning.java` | Both | Whether the Driver Station preview starts enabled. | TeleOp also exposes Gamepad 2 D-pad up/down to toggle live view at runtime; keep default `false` for performance. | Set `true` when tethered tuning sessions prefer immediate preview. |
| `TeleOpDriverDefaults.TELEMETRY_SMOOTH_A` | `config/TeleOpDriverDefaults.java` | TeleOp | Smoothing for range & heading telemetry. | TeleOp-only telemetry; Auto ignores it. | `0.4` to calm noisy readouts; `0.15` for responsiveness. |

## Drivetrain motion & positioning

| Parameter | Set in | Impacts | What it controls | Tune here vs. elsewhere | Sample adjustments |
| --- | --- | --- | --- | --- | --- |
| `SharedRobotTuning.DRIVE_MAX_POWER` | `config/SharedRobotTuning.java` | Auto | Power cap for auto translation helpers. | Overrides `BaseAuto.driveCap()` (replacing `DEF_DRIVE_CAP`). Tune here for global auto speed changes. | `0.65` when traction allows; `0.40` on slick tiles. |
| `DriveTuning.WHEEL_DIAMETER_IN` | `config/DriveTuning.java` | Both | Wheel circumference used for distance math. | Impacts odometry everywhere; keep synced with hardware. | `4.0 in` when swapping to 4″ wheels. |
| `DriveTuning.TICKS_PER_REV` | `config/DriveTuning.java` | Both | Encoder ticks per motor revolution. | Ensure matches installed cartridges to avoid drift. | `383.6` for 435 RPM goBILDA motors. |
| `DriveTuning.GEAR_RATIO` | `config/DriveTuning.java` | Both | External gear ratio applied to wheels. | Change when drivetrain gearing changes. | `1.5` for 3:2 gearing. |
| `DriveTuning.STRAFE_CORRECTION` | `config/DriveTuning.java` | Both | Multiplier correcting mecanum strafing undershoot. | Shared for TeleOp strafing and Auto pathing; tune based on field tests. | Between `1.05–1.25` depending on carpet. |
| `DriveTuning.TURN_KP` / `TURN_KD` | `config/DriveTuning.java` | Both | PD gains for IMU-based turns. | TeleOp brake-twist and Auto turns share these; adjust to taste. | Raise `KP` to `0.016` for quicker snaps; add `KD` up to `0.005` to reduce overshoot. |
| `DriveTuning.TURN_SETTLE_TIME_SEC` | `config/DriveTuning.java` | Both | Time inside tolerance before declaring a turn done. | Applies to any IMU turn helper; Auto benefits from extra stability, TeleOp from smoother braking. | `0.10 s` for faster autos; `0.20 s` if oscillating. |
| `SharedRobotTuning.LOGO_DIRECTION` / `USB_DIRECTION` | `config/SharedRobotTuning.java` | Both | REV Hub IMU mounting orientation. | Drivebase pulls both values when initializing the IMU; update here whenever the hub is remounted so yaw math stays correct. | `LOGO UP` / `USB RIGHT` for the FTC standard; flip to match new mounting. |

## Autonomous routine pacing

| Parameter | Set in | Impacts | What it controls | Tune here vs. elsewhere | Sample adjustments |
| --- | --- | --- | --- | --- | --- |
| `BaseAuto.turnToGoalTag(timeoutMs)` | `auto/BaseAuto.java` & derived classes | Auto | Timeout for acquiring a tag and aligning. | Each auto class passes its own timeout; if a derived class overrides the method, adjust there first. Shared turn cap & tolerances still come from `SharedRobotTuning`. | `2500 ms` when tags are harder to find. |
| `BaseAuto.aimSpinUntilReady(timeoutMs)` | `auto/BaseAuto.java` & derived classes | Auto | Spin-up guard while waiting for readiness. | Derived classes set the timeout; cadence is governed by `SharedRobotTuning` shot delay/tolerance. | Extend to `3200 ms` if flywheel needs longer to reach speed. |
| `BaseAuto.driveForwardInches(distance)` & other `drive` helpers | `auto/Auto_*` routines | Auto | Distances for stage positioning. | Change inside the routine that owns the move. Power cap comes from `SharedRobotTuning.DRIVE_MAX_POWER`. | Shorten setup drive to `30"` for closer standoff. |

## Intake power & driver defaults

| Parameter | Set in | Impacts | What it controls | Tune here vs. elsewhere | Sample adjustments |
| --- | --- | --- | --- | --- | --- |
| `IntakeTuning.POWER_ON` | `config/IntakeTuning.java` | Both | Motor power while intake is active. | Single source for Auto and TeleOp. | `0.9` for faster capture; `0.6` to prevent jams. |
| `TeleOpDriverDefaults.INTAKE_ENABLED` | `config/TeleOpDriverDefaults.java` | TeleOp | Whether TeleOp starts with intake running. | TeleOp-only driver preference. | Defaults to `true`; safeInit keeps the motor idle during INIT if you prefer to toggle manually. |
| `TeleOpDriverDefaults.AUTO_SPEED_ENABLED` | `config/TeleOpDriverDefaults.java` | TeleOp | Whether AutoSpeed is enabled at init. | TeleOp only; Auto uses AutoSpeed automatically when requested. | Defaults to `true`; flip to `false` if drivers want to spool manually after START. |
| `TeleOpDriverDefaults.AUTO_AIM_ENABLED` | `config/TeleOpDriverDefaults.java` | TeleOp | Whether AutoAim starts enabled. | TeleOp only. | `true` when drivers rely on aim assist immediately. |
| `TeleOpDriverDefaults.AUTO_STOP_TIMER_ENABLED` / `AUTO_STOP_TIMER_TIME_SEC` | `config/TeleOpDriverDefaults.java` | TeleOp | Optional end-of-match safety timer. | No impact on Auto; tune per event policy. | Enable with `115 s` to stop before endgame. |
| `TeleOpDriverDefaults.SLOWEST_SPEED` | `config/TeleOpDriverDefaults.java` | TeleOp | Minimum drive speed with brake trigger held. | TeleOp-only brake scaling. | `0.15` for tight endgame alignment; `0.35` if too sluggish. |
| `TeleOpDriverDefaults.AUTO_AIM_LOSS_GRACE_MS` | `config/TeleOpDriverDefaults.java` | TeleOp | How long AutoAim remains latched after losing a tag. | TeleOp only. | `2500 ms` for faster manual fallback. |
| `TeleOpDriverDefaults.RPM_TEST_STEP` | `config/TeleOpDriverDefaults.java` | TeleOp | D-pad RPM step size in test mode. | TeleOp-only diagnostic; unrelated to Auto. | Change to `±25 RPM` for finer sweeps. |

## Driver feedback (rumble & haptics)

| Parameter | Set in | Impacts | What it controls | Tune here vs. elsewhere | Sample adjustments |
| --- | --- | --- | --- | --- | --- |
| `TeleOpRumbleTuning.AIM_RUMBLE_ENABLED` | `config/TeleOpRumbleTuning.java` | TeleOp | Global enable for manual aim rumble. | TeleOp flips the shared `RumbleNotifier`; Auto never engages it. | Set `false` if haptics distract drivers. |
| `TeleOpRumbleTuning.AIM_THRESHOLD_DEG` | `config/TeleOpRumbleTuning.java` | TeleOp | Heading error threshold to start rumble. | Overrides `RumbleNotifier.thresholdDeg`. Tune here for TeleOp preference; Auto doesn’t rumble. | `1.5°` when drivers want earlier cues. |
| `TeleOpRumbleTuning.AIM_STRENGTH_MIN` / `AIM_STRENGTH_MAX` | `config/TeleOpRumbleTuning.java` | TeleOp | Strength bounds for aim rumble. | Override `RumbleNotifier` min/max. TeleOp only. | `0.05–0.80` for a wider range. |
| `TeleOpRumbleTuning.AIM_PULSE_MIN_MS` / `AIM_PULSE_MAX_MS` | `config/TeleOpRumbleTuning.java` | TeleOp | Pulse duration range for aim rumble. | Overrides notifier defaults. | `80–160 ms` for crisp pulses. |
| `TeleOpRumbleTuning.AIM_COOLDOWN_MIN_MS` / `AIM_COOLDOWN_MAX_MS` | `config/TeleOpRumbleTuning.java` | TeleOp | Cooldown range between pulses. | Overrides notifier defaults. | `60–240 ms` when cues feel too sparse. |
| `TeleOpRumbleTuning.TOGGLE_STRENGTH` / `TOGGLE_STEP_MS` / `TOGGLE_GAP_MS` | `config/TeleOpRumbleTuning.java` | TeleOp | Rumble feedback for mode toggles. | TeleOp-specific; does not touch shared notifier. | `0.6 strength` if controllers saturate; `90/60 ms` for snappier toggles. |
| `utils/RumbleNotifier.thresholdDeg` / `minStrength` / `maxStrength` / `minPulseMs` / `maxPulseMs` / `minCooldownMs` / `maxCooldownMs` | `utils/RumbleNotifier.java` | TeleOp (when not overridden) | Default rumble characteristics. | Immediately overwritten by TeleOp aim tunables above; edit here only if you need new global defaults before TeleOp touches them. | Raise defaults when using notifier for other features. |

## Controller interface & miscellaneous utilities

| Parameter | Set in | Impacts | What it controls | Tune here vs. elsewhere | Sample adjustments |
| --- | --- | --- | --- | --- | --- |
| `ControllerTuning.TRIGGER_EDGE_THRESH` | `config/ControllerTuning.java` | TeleOp | Trigger value treated as a button press. | Affects TeleOp trigger→button mappings. No overrides elsewhere. | `0.3` for lighter pulls; `0.6` for deliberate presses. |
| `TeleOpAllianceBase.aimRumbleEnabled` (see above) | `teleop/TeleOpAllianceBase.java` | TeleOp | Doubles as feedback toggle for miscellaneous alerts. | Keep aligned with other rumble settings. | — |
| `assist/AutoAimSpeed.enable()` seeds | `assist/AutoAimSpeed.java` | Both | Syncs helper fields with shared tuning when enabling AutoAim. | Pulls from `SharedRobotTuning`; if TeleOp overrides `InitialAutoDefaultSpeed`, also set `autoAimSpeed.initialAutoDefaultSpeed` to match. | Call `autoAimSpeed.initialAutoDefaultSpeed = 2400` after changing TeleOp copy. |

---

Whenever you change anything that touches distance, speed curves, or AprilTag alignment, re-validate Autonomous and TeleOp together so launcher RPM, drive odometry, and aiming assumptions stay consistent across the field.

