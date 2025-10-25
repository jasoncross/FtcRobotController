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
| `AutoRpmConfig.NEAR_DIST_IN` / `NEAR_RPM` / `FAR_DIST_IN` / `FAR_RPM` | `config/AutoRpmConfig.java` | Both | Anchor points for the AutoSpeed distance→RPM curve. | This file overwrites `LauncherAutoSpeedController` defaults every time `AutoRpmConfig.apply(...)` runs (TeleOp init + Auto helpers). Edit here unless you need a one-off experiment in a fork. | Shift to `30 in → 1200 RPM` when near shots dip; `110 in → 4300 RPM` if far shots sail. |
| `AutoRpmConfig.SMOOTH_ALPHA` | `config/AutoRpmConfig.java` | Both | Exponential smoothing applied to RPM updates. | Overrides the controller’s internal `smoothingAlpha`. Change here to keep Auto and TeleOp aligned. | `0.05` for snappier response, `0.25` to calm oscillations. |
| `LauncherAutoSpeedController.nearDistanceIn` / `nearSpeedRpm` / `farDistanceIn` / `farSpeedRpm` | `control/LauncherAutoSpeedController.java` | Both | Backup defaults for the AutoSpeed curve. | Only take effect if `AutoRpmConfig.apply(...)` is skipped. Tune via `AutoRpmConfig` unless you are intentionally bypassing it. | Use curved anchors for lab testing before copying into `AutoRpmConfig`. |
| `LauncherAutoSpeedController.smoothingAlpha` | `control/LauncherAutoSpeedController.java` | Both | Controller-local smoothing constant. | Immediately replaced by `AutoRpmConfig.SMOOTH_ALPHA`; keep equal so local tests mirror real play. | Match `AutoRpmConfig` while experimenting. |
| `SharedRobotTuning.INITIAL_AUTO_DEFAULT_SPEED` | `config/SharedRobotTuning.java` | Both (seed) | Starting RPM before the first AprilTag lock. | Central value feeds Auto helpers and `assist/AutoAimSpeed`; TeleOp copies it to `TeleOpAllianceBase.InitialAutoDefaultSpeed`. Change it here unless you need a TeleOp-only divergence. | `2800 RPM` for long fields; `2200 RPM` for short volleys. |
| `teleop/TeleOpAllianceBase.InitialAutoDefaultSpeed` | `teleop/TeleOpAllianceBase.java` | TeleOp | Local override of the seed RPM. | Overrides TeleOp only; Autonomous and `AutoAimSpeed` stay on the shared value. Only edit when drivers want a different opening RPM than Auto. | `2400 RPM` if TeleOp wants faster spin-up than Auto. |
| `Launcher.RPM_MIN` / `RPM_MAX` | `subsystems/Launcher.java` | Both | Software clamp on any RPM request. | Applies after all other tunables. Lowering `RPM_MAX` here limits both AutoSpeed and manual RPM. | Drop max to `5500` if motors saturate; raise min to `200` to keep wheels spinning. |
| `Launcher.PIDF` (`P`, `I`, `D`, `F`) | `subsystems/Launcher.java` | Both | Velocity loop gains sent to the REV Hub. | Shared across all modes; tune here so Auto and TeleOp remain stable together. | Increase `P` to `12` for heavier wheels; add `D≈0.5` when overshooting. |
| `Launcher.atSpeedToleranceRPM` | `subsystems/Launcher.java` | Both | Launcher’s own “ready” window. | Used when no custom tolerance is provided; coordinate with `SharedRobotTuning.RPM_TOLERANCE`. | Tighten to `60 RPM` for accuracy; loosen to `120 RPM` if jittery. |
| `teleop/TeleOpAllianceBase.rpmBottom` / `rpmTop` | `teleop/TeleOpAllianceBase.java` | TeleOp | Manual-mode RPM slider limits. | Independent of the launcher clamp; ensure `rpmTop ≤ Launcher.RPM_MAX`. | Set bottom to `500 RPM` to keep wheels warm; lower top to `5500 RPM` if browning out. |
| `SharedRobotTuning.RPM_TOLERANCE` | `config/SharedRobotTuning.java` | Both | Global readiness window for AutoSpeed. | Overrides `BaseAuto.rpmTol()` and seeds `assist/AutoAimSpeed.rpmTolerance`; TeleOp follows unless it overrides locally. Tune here for shared behavior. | `35 RPM` for precise volleys; `75 RPM` if batteries sag. |
| `assist/AutoAimSpeed.rpmTolerance` | `assist/AutoAimSpeed.java` | Both | AutoAim-specific readiness window. | Defaults to `SharedRobotTuning.RPM_TOLERANCE`; overriding here affects AutoAim only, not the rest of AutoSpeed usage. | Narrow to `30 RPM` when AutoAim needs stricter gating. |

## Shot cadence, feed, and eject

| Parameter | Set in | Impacts | What it controls | Tune here vs. elsewhere | Sample adjustments |
| --- | --- | --- | --- | --- | --- |
| `SharedRobotTuning.SHOT_BETWEEN_MS` | `config/SharedRobotTuning.java` | Auto | Minimum delay between autonomous shots. | Replaces `BaseAuto.DEF_BETWEEN_MS`; adjust here for global cadence changes. | `2500 ms` for faster volleys once feed is proven. |
| `SharedRobotTuning.INTAKE_ASSIST_MS` | `config/SharedRobotTuning.java` | Auto (direct) / TeleOp (via copy) | How long intake runs after a feed when previously off. | Central value drives Auto; TeleOp copies to `TeleOpAllianceBase.intakeAssistMs`. Change centrally unless you need a TeleOp-specific tweak. | `350 ms` for sticky rings; `200 ms` when intake clears fast. |
| `teleop/TeleOpAllianceBase.intakeAssistMs` | `teleop/TeleOpAllianceBase.java` | TeleOp | TeleOp-only override of intake assist. | Overrides TeleOp behavior without touching Auto. Keep aligned with the shared value unless driver workflows demand otherwise. | Extend to `320 ms` if TeleOp wants extra assurance. |
| `subsystems/Feed.firePower` | `subsystems/Feed.java` | Both | Motor power used to push a ring. | Shared by Auto `fireN` and TeleOp feed buttons. | Bump to `1.0` when rings stick; lower to `0.75` to reduce jams. |
| `subsystems/Feed.fireTimeMs` | `subsystems/Feed.java` | Both | Time the feed motor runs per shot. | Single source for Auto + TeleOp. | `450 ms` after tightening timing; `650 ms` if rings hesitate. |
| `subsystems/Feed.minCycleMs` | `subsystems/Feed.java` | Both | Cooldown between shots. | Keep consistent so Auto and TeleOp pacing match. | `200 ms` when hardware tolerates quick cycles. |
| `teleop/TeleOpAllianceBase.ejectRpm` | `teleop/TeleOpAllianceBase.java` | TeleOp | Launcher RPM during eject routine. | TeleOp-only; Auto never calls eject. Tune here without affecting Auto. | `400 RPM` for gentle clears; `800 RPM` for stubborn jams. |
| `teleop/TeleOpAllianceBase.ejectTimeMs` | `teleop/TeleOpAllianceBase.java` | TeleOp | Duration of eject routine. | TeleOp only. | `600 ms` for quick clear, `1400 ms` for heavy debris. |

## AutoAim, targeting, and AprilTag alignment

| Parameter | Set in | Impacts | What it controls | Tune here vs. elsewhere | Sample adjustments |
| --- | --- | --- | --- | --- | --- |
| `SharedRobotTuning.TURN_TWIST_CAP` | `config/SharedRobotTuning.java` | Both | Maximum twist power for automatic aiming/turns. | Overrides `BaseAuto.turnTwistCap()` and seeds `assist/AutoAimSpeed.maxTwist`. Tune centrally for consistent feel. | Raise to `0.45` for aggressive steering; drop to `0.25` if overshooting. |
| `assist/AutoAimSpeed.maxTwist` | `assist/AutoAimSpeed.java` | Both | AutoAim-specific twist clamp. | Defaults to the shared cap; override here only if AutoAim needs a different cap than Auto turn helpers. | Increase slightly above shared cap for faster driver assists. |
| `SharedRobotTuning.LOCK_TOLERANCE_DEG` | `config/SharedRobotTuning.java` | Auto | Acceptable AprilTag bearing error before firing. | Overrides `BaseAuto.lockTolDeg()` (replacing `DEF_LOCK_TOL_DEG`). Tune here to keep all auto routines consistent. | `0.5°` for precise shots; `1.5°` if heading drifts. |
| `drive/Drivebase.TURN_TOLERANCE_DEG` | `drive/Drivebase.java` | Both | IMU turn completion tolerance. | Works with `LOCK_TOLERANCE_DEG`; keep both aligned. TeleOp braking also uses this. | Tighten to `0.5°` when Auto must be exact. |
| `SharedRobotTuning.TURN_TWIST_CAP` + `TagAimController` clamp | `vision/TagAimController.java` | Both | Twist limits inside the AprilTag controller. | Ensure the clamp (default ±`0.6`) is ≥ shared cap so AutoAim isn’t double-limited. Edit clamp if you need more headroom. | Expand to `±0.8` for faster centering. |
| `TagAimController.kP` / `kD` | `vision/TagAimController.java` | Both | PD gains for tag-based twist corrections. | Shared by AutoAim and BaseAuto; tune here for consistent tracking. | `0.025/0.004` for quicker reaction; dial back `kD` if noisy. |
| `SharedRobotTuning.RPM_TOLERANCE` (see above) | `config/SharedRobotTuning.java` | Both | Launch readiness gating for AutoAim + Auto. | Shared reference for all aim subsystems; coordinate with `Launcher.atSpeedToleranceRPM`. | As above. |
| `assist/AutoAimSpeed.initialAutoDefaultSpeed` | `assist/AutoAimSpeed.java` | Both | AutoAim’s seed RPM before first tag fix. | Mirrors `SharedRobotTuning.INITIAL_AUTO_DEFAULT_SPEED`; override here if AutoAim needs a unique seed. | Match TeleOp override when diverging from shared value. |

## Vision & range calibration

| Parameter | Set in | Impacts | What it controls | Tune here vs. elsewhere | Sample adjustments |
| --- | --- | --- | --- | --- | --- |
| `VisionAprilTag.setRangeScale(...)` | `vision/VisionAprilTag.java` (called from `TeleOpAllianceBase.init`) | Both (shared instance) | Multiplier that converts camera-estimated distance to real inches. | TeleOp initializes the shared `VisionAprilTag` instance with `0.03`, overriding the class default of `1.0`. Updating the scale affects any mode using that instance (Auto, AutoAim). | Calibrate by `trueDistance / measuredDistance`; typically near `1.0`. |
| `TeleOpAllianceBase.SMOOTH_A` | `teleop/TeleOpAllianceBase.java` | TeleOp | Smoothing for range & heading telemetry. | TeleOp-only telemetry; Auto ignores it. | `0.4` to calm noisy readouts; `0.15` for responsiveness. |

## Drivetrain motion & positioning

| Parameter | Set in | Impacts | What it controls | Tune here vs. elsewhere | Sample adjustments |
| --- | --- | --- | --- | --- | --- |
| `SharedRobotTuning.DRIVE_MAX_POWER` | `config/SharedRobotTuning.java` | Auto | Power cap for auto translation helpers. | Overrides `BaseAuto.driveCap()` (replacing `DEF_DRIVE_CAP`). Tune here for global auto speed changes. | `0.65` when traction allows; `0.40` on slick tiles. |
| `drive/Drivebase.WHEEL_DIAMETER_IN` | `drive/Drivebase.java` | Both | Wheel circumference used for distance math. | Impacts odometry everywhere; keep synced with hardware. | `4.0 in` when swapping to 4″ wheels. |
| `drive/Drivebase.TICKS_PER_REV` | `drive/Drivebase.java` | Both | Encoder ticks per motor revolution. | Ensure matches installed cartridges to avoid drift. | `383.6` for 435 RPM goBILDA motors. |
| `drive/Drivebase.GEAR_RATIO` | `drive/Drivebase.java` | Both | External gear ratio applied to wheels. | Change when drivetrain gearing changes. | `1.5` for 3:2 gearing. |
| `drive/Drivebase.STRAFE_CORRECTION` | `drive/Drivebase.java` | Both | Multiplier correcting mecanum strafing undershoot. | Shared for TeleOp strafing and Auto pathing; tune based on field tests. | Between `1.05–1.25` depending on carpet. |
| `drive/Drivebase.TURN_KP` / `TURN_KD` | `drive/Drivebase.java` | Both | PD gains for IMU-based turns. | TeleOp brake-twist and Auto turns share these; adjust to taste. | Raise `KP` to `0.016` for quicker snaps; add `KD` up to `0.005` to reduce overshoot. |
| `drive/Drivebase.TURN_SETTLE_TIME` | `drive/Drivebase.java` | Both | Time inside tolerance before declaring a turn done. | Applies to any IMU turn helper; Auto benefits from extra stability, TeleOp from smoother braking. | `0.10 s` for faster autos; `0.20 s` if oscillating. |

## Autonomous routine pacing

| Parameter | Set in | Impacts | What it controls | Tune here vs. elsewhere | Sample adjustments |
| --- | --- | --- | --- | --- | --- |
| `BaseAuto.turnToGoalTag(timeoutMs)` | `auto/BaseAuto.java` & derived classes | Auto | Timeout for acquiring a tag and aligning. | Each auto class passes its own timeout; if a derived class overrides the method, adjust there first. Shared turn cap & tolerances still come from `SharedRobotTuning`. | `2500 ms` when tags are harder to find. |
| `BaseAuto.aimSpinUntilReady(timeoutMs)` | `auto/BaseAuto.java` & derived classes | Auto | Spin-up guard while waiting for readiness. | Derived classes set the timeout; cadence is governed by `SharedRobotTuning` shot delay/tolerance. | Extend to `3200 ms` if flywheel needs longer to reach speed. |
| `BaseAuto.driveForwardInches(distance)` & other `drive` helpers | `auto/Auto_*` routines | Auto | Distances for stage positioning. | Change inside the routine that owns the move. Power cap comes from `SharedRobotTuning.DRIVE_MAX_POWER`. | Shorten setup drive to `30"` for closer standoff. |

## Intake power & driver defaults

| Parameter | Set in | Impacts | What it controls | Tune here vs. elsewhere | Sample adjustments |
| --- | --- | --- | --- | --- | --- |
| `subsystems/Intake.powerOn` | `subsystems/Intake.java` | Both | Motor power while intake is active. | Single source for Auto and TeleOp. | `0.9` for faster capture; `0.6` to prevent jams. |
| `TeleOpAllianceBase.DEFAULT_INTAKE_ENABLED` | `teleop/TeleOpAllianceBase.java` | TeleOp | Whether TeleOp starts with intake running. | TeleOp-only driver preference. | Set `true` when wanting instant cycling. |
| `TeleOpAllianceBase.DEFAULT_AUTOSPEED_ENABLED` | `teleop/TeleOpAllianceBase.java` | TeleOp | Whether AutoSpeed is enabled at init. | TeleOp only; Auto uses AutoSpeed automatically when requested. | `true` when AutoSpeed is always desired. |
| `TeleOpAllianceBase.DEFAULT_AUTOAIM_ENABLED` | `teleop/TeleOpAllianceBase.java` | TeleOp | Whether AutoAim starts enabled. | TeleOp only. | `true` when drivers rely on aim assist immediately. |
| `TeleOpAllianceBase.autoStopTimerEnabled` / `autoStopTimerTimeSec` | `teleop/TeleOpAllianceBase.java` | TeleOp | Optional end-of-match safety timer. | No impact on Auto; tune per event policy. | Enable with `115 s` to stop before endgame. |
| `TeleOpAllianceBase.slowestSpeed` | `teleop/TeleOpAllianceBase.java` | TeleOp | Minimum drive speed with brake trigger held. | TeleOp-only brake scaling. | `0.15` for tight endgame alignment; `0.35` if too sluggish. |
| `TeleOpAllianceBase.autoAimLossGraceMs` | `teleop/TeleOpAllianceBase.java` | TeleOp | How long AutoAim remains latched after losing a tag. | TeleOp only. | `2500 ms` for faster manual fallback. |
| Manual RPM test increment (`±50 RPM`) | `teleop/TeleOpAllianceBase.java` | TeleOp | D-pad RPM step size in test mode. | TeleOp-only diagnostic; unrelated to Auto. | Change to `±25 RPM` for finer sweeps. |

## Driver feedback (rumble & haptics)

| Parameter | Set in | Impacts | What it controls | Tune here vs. elsewhere | Sample adjustments |
| --- | --- | --- | --- | --- | --- |
| `TeleOpAllianceBase.aimRumbleEnabled` | `teleop/TeleOpAllianceBase.java` | TeleOp | Global enable for manual aim rumble. | TeleOp flips the shared `RumbleNotifier`; Auto never engages it. | Set `false` if haptics distract drivers. |
| `TeleOpAllianceBase.aimRumbleDeg` | `teleop/TeleOpAllianceBase.java` | TeleOp | Heading error threshold to start rumble. | Overrides `RumbleNotifier.thresholdDeg`. Tune here for TeleOp preference; Auto doesn’t rumble. | `1.5°` when drivers want earlier cues. |
| `TeleOpAllianceBase.aimRumbleMinStrength` / `aimRumbleMaxStrength` | `teleop/TeleOpAllianceBase.java` | TeleOp | Strength bounds for aim rumble. | Override `RumbleNotifier` min/max. TeleOp only. | `0.05–0.80` for a wider range. |
| `TeleOpAllianceBase.aimRumbleMinPulseMs` / `aimRumbleMaxPulseMs` | `teleop/TeleOpAllianceBase.java` | TeleOp | Pulse duration range for aim rumble. | Overrides notifier defaults. | `80–160 ms` for crisp pulses. |
| `TeleOpAllianceBase.aimRumbleMinCooldownMs` / `aimRumbleMaxCooldownMs` | `teleop/TeleOpAllianceBase.java` | TeleOp | Cooldown range between pulses. | Overrides notifier defaults. | `60–240 ms` when cues feel too sparse. |
| `TeleOpAllianceBase.togglePulseStrength` / `togglePulseStepMs` / `togglePulseGapMs` | `teleop/TeleOpAllianceBase.java` | TeleOp | Rumble feedback for mode toggles. | TeleOp-specific; does not touch shared notifier. | `0.6 strength` if controllers saturate; `90/60 ms` for snappier toggles. |
| `utils/RumbleNotifier.thresholdDeg` / `minStrength` / `maxStrength` / `minPulseMs` / `maxPulseMs` / `minCooldownMs` / `maxCooldownMs` | `utils/RumbleNotifier.java` | TeleOp (when not overridden) | Default rumble characteristics. | Immediately overwritten by TeleOp aim tunables above; edit here only if you need new global defaults before TeleOp touches them. | Raise defaults when using notifier for other features. |

## Controller interface & miscellaneous utilities

| Parameter | Set in | Impacts | What it controls | Tune here vs. elsewhere | Sample adjustments |
| --- | --- | --- | --- | --- | --- |
| `input/ControllerBindings.TRIGGER_EDGE_THRESH` | `input/ControllerBindings.java` | TeleOp | Trigger value treated as a button press. | Affects TeleOp trigger→button mappings. No overrides elsewhere. | `0.3` for lighter pulls; `0.6` for deliberate presses. |
| `TeleOpAllianceBase.aimRumbleEnabled` (see above) | `teleop/TeleOpAllianceBase.java` | TeleOp | Doubles as feedback toggle for miscellaneous alerts. | Keep aligned with other rumble settings. | — |
| `assist/AutoAimSpeed.enable()` seeds | `assist/AutoAimSpeed.java` | Both | Syncs helper fields with shared tuning when enabling AutoAim. | Pulls from `SharedRobotTuning`; if TeleOp overrides `InitialAutoDefaultSpeed`, also set `autoAimSpeed.initialAutoDefaultSpeed` to match. | Call `autoAimSpeed.initialAutoDefaultSpeed = 2400` after changing TeleOp copy. |

---

Whenever you change anything that touches distance, speed curves, or AprilTag alignment, re-validate Autonomous and TeleOp together so launcher RPM, drive odometry, and aiming assumptions stay consistent across the field.

