package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.FeedStopConfig;
import org.firstinspires.ftc.teamcode.config.FeedTuning;

import java.util.Locale;

/*
 * FILE: Feed.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/
 *
 * PURPOSE
 *   - Drive the single feed motor that pushes one ARTIFACT into the launcher per
 *     cycle for both TeleOp and Autonomous.
 *   - Provide a timed, debounce-guarded routine so BaseAuto.fireN() and TeleOp
 *     buttons share identical cadence logic.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Shot cadence, feed, and eject)
 *   - firePower
 *       • Motor power applied during the feed pulse.
 *       • Shared across modes; increase toward 1.0 when artifacts stick, lower toward
 *         0.7 if jams occur.
 *   - fireTimeMs
 *       • Duration of the feed pulse in milliseconds (450–650 typical).
 *       • Confirm autonomous sequences leave enough recovery time between
 *         feedOnceBlocking() calls for the flywheels to stay at speed.
 *   - minCycleMs
 *       • Minimum delay between feeds to prevent double-fires.
 *       • Keep aligned with TeleOpAllianceBase button debounce expectations.
 *
 * METHODS
 *   - canFire()
 *       • Debounce check used before calling feedOnceBlocking().
 *   - feedOnceBlocking()
 *       • Timed feed routine shared by Auto/TeleOp.
 *   - stop() / setPower()
 *       • Safety helpers used by StopAll and diagnostics.
 *
 * NOTES
 *   - Motor uses BRAKE zero-power behavior so the pusher stays loaded against the
 *     artifact stack when idle.
 *   - Future encoder-based feeds can extend this class by replacing the timed
 *     sleep with RUN_TO_POSITION logic.
 */
public class Feed {

    // CHANGES (2025-10-30): Locked zero-power BRAKE + RUN_WITHOUT_ENCODER and guard before each power command.
    // CHANGES (2025-10-31): Added idle counter-rotation via FeedTuning.IDLE_HOLD_POWER when not firing.
    // CHANGES (2025-10-31): Added safeInit + idle hold gating so motors stay idle until START.
    // CHANGES (2025-11-02): Clarified cadence guidance now that shot spacing is per AutoSequence.
    // CHANGES (2025-11-04): Added applyBrakeHold() so StopAll explicitly latches BRAKE with zero power.
    // CHANGES (2025-11-06): Integrated FeedStop servo gate with request/hold timing + tunables.
    // CHANGES (2025-11-07): Converted feedOnce() into a non-blocking state machine for TeleOp, while
    //                       keeping the blocking helper for Auto; added FeedStop-aware async cycle
    //                       handling so TeleOp no longer sleeps the main loop when feeding.
    // CHANGES (2025-11-07): Added FeedStop homing with degrees-based targets, stop-to-zero behavior,
    //                       and cleaned up obsolete FeedStop tunables.
    // CHANGES (2025-11-08): Introduced separate hold/release degree targets, auto-expanded the servo
    //                       scale window with safety margin math, and ensured StopAll parks at the
    //                       homed zero while runtime cycles return to the hold angle.
    // CHANGES (2025-11-09): Default to full-span servo mapping, added optional auto-scaling, and
    //                       refreshed telemetry/clamping rules for degree-based control.
    // CHANGES (2025-11-07): Added guarded FeedStop homing with soft-limit clamps so the servo never
    //                       drives past 0° or beyond the upper linkage range, plus telemetry for
    //                       homing aborts and clamped requests.
    // CHANGES (2025-11-09): Condensed FeedStop telemetry helpers into a single summary line with
    //                       warnings surfaced only when limits or homing guards trigger.
    // CHANGES (2025-11-23): Added continuous-feed mode for fire-button holds so TeleOp can stream
    //                       artifacts without rearming the state machine between shots.
    // CHANGES (2025-11-29): Exposed releaseHoldMs so TeleOp can delay continuous-feed conversion
    //                       until a deliberate hold after the standard single-shot window.
    // CHANGES (2025-12-16): Deferred FeedStop homing/parking until START so INIT remains motionless
    //                       while still queuing homing for the first post-START loops.
    // CHANGES (2025-12-31): Started the FeedStop return timer only when the feed motor begins
    //                       moving so the gate stays open until the shot actually fires.
    // CHANGES (2025-12-31): Clarified feed gating uses a ±RPM readiness window so overspeed is
    //                       treated the same as underspeed.
    public double firePower = FeedTuning.FIRE_POWER; // Shared motor power; referenced by BaseAuto.fireN() + TeleOp bindings
    public int fireTimeMs   = FeedTuning.FIRE_TIME_MS;  // Duration of each feed pulse (ms); ensure sequences allow recovery time
    public int minCycleMs   = FeedTuning.MIN_CYCLE_MS;  // Minimum delay between feeds; prevents double-fire even if buttons spammed
    public double idleHoldPower = FeedTuning.IDLE_HOLD_POWER; // Counter-rotation while idle (0 = BRAKE only)

    private final DcMotorEx motor;
    private long lastFire = 0;
    private boolean idleHoldActive = false;

    private static final double FULL_SPAN_DEG = 300.0;

    private ServoImplEx feedStop;
    private Telemetry feedStopTelemetry;
    private boolean feedStopReady = false;
    private FeedStopState feedStopState = FeedStopState.UNKNOWN;
    private double directionSign = 1.0;
    private double homeBackoffDeg = 0.0;
    private long homeDwellMs = 0L;
    private double configuredHoldAngleDeg = 0.0;
    private double configuredReleaseAngleDeg = 0.0;
    private double safetyMarginDeg = 0.0;
    private double holdAngleDeg = 0.0;
    private double releaseAngleDeg = 0.0;
    private double softCcwLimitDeg = 0.0;
    private double softCwLimitDeg = 0.0;
    private double safePresetOpenDeg = 0.0;
    private double maxHomeTravelDeg = 0.0;
    private long releaseHoldMs = 0L;
    private long fireLeadMs = 0L;
    private long releaseUntilMs = 0L;
    private long feedAllowedAfterMs = 0L;
    private boolean continuousFeedActive = false;
    private boolean feedReadyGate = true;

    private boolean useAutoScale = false;
    private boolean autoScaleApplied = false;
    private double appliedScaleMin = Double.NaN;
    private double appliedScaleMax = Double.NaN;
    private double windowDegrees = FULL_SPAN_DEG;
    private double requiredSpanDeg = 0.0;
    private double maxAvailableAngleDeg = FULL_SPAN_DEG;
    private double zeroPosition = Double.NaN;
    private double holdPosition = Double.NaN;
    private double releasePosition = Double.NaN;
    private double homeCommandedDeg = 0.0;
    private double homeTargetDeg = 0.0;
    private double homeTravelAccumDeg = 0.0;
    private long lastHomeStepMs = 0L;
    private boolean feedStopHomed = false;
    private boolean windowLimitReached = false;
    private boolean angleClamped = false;
    private boolean softLimitClamped = false;
    private boolean scaleTelemetryEmitted = false;
    private boolean homeAborted = false;
    private String homeAbortMessage = null;
    private String softLimitMessage = null;

    private boolean feedStopHomeQueued = false;

    private HomeState homeState = HomeState.IDLE;
    private long homeStateStartMs = 0L;

    private FeedCycleState cycleState = FeedCycleState.IDLE;
    private long cycleStateStartMs = 0L;

    private enum FeedCycleState {
        IDLE,
        WAIT_FOR_LEAD,
        FEEDING
    }

    private enum HomeState {
        IDLE,
        SAFE_OPEN_INIT,
        SAFE_OPEN_STEP,
        MOVE_TO_ZERO,
        DWELL_AT_ZERO,
        BACKOFF_FROM_ZERO,
        ABORTED,
        COMPLETE
    }

    private static final double HOME_STEP_DEG = 5.0;
    private static final long HOME_STEP_INTERVAL_MS = 60L;

    public Feed(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, "FeedMotor");
        applySafetyConfig();
        safeInit();
    }

    /** Ensure the feed holds zero power during INIT (no motion before START). */
    public void safeInit() {
        applySafetyConfig();
        idleHoldActive = false;
        motor.setPower(0.0);
        feedStopHomed = false;
        homeState = HomeState.IDLE;
        homeStateStartMs = 0L;
        lastHomeStepMs = 0L;
        homeCommandedDeg = 0.0;
        homeTargetDeg = 0.0;
        homeTravelAccumDeg = 0.0;
        homeAborted = false;
        homeAbortMessage = null;
        zeroPosition = Double.NaN;
        holdPosition = Double.NaN;
        releasePosition = Double.NaN;
        releaseUntilMs = 0L;
        feedAllowedAfterMs = 0L;
        continuousFeedActive = false;
        feedReadyGate = true;
        scaleTelemetryEmitted = false;
        windowLimitReached = false;
        angleClamped = false;
        softLimitClamped = false;
        softLimitMessage = null;
        feedStopHomeQueued = false;
        useAutoScale = false;
        autoScaleApplied = false;
        applyFeedStopConfigValues();
        cycleState = FeedCycleState.IDLE;
        cycleStateStartMs = 0L;
    }

    /** Enable or disable the idle hold counter-rotation after START. */
    public void setIdleHoldActive(boolean enable) {
        idleHoldActive = enable;
        applyIdleHoldPower();
    }

    /** Configure the FeedStop servo, scaling its range and parking at BLOCK. */
    public void initFeedStop(HardwareMap hw, Telemetry telemetry) {
        this.feedStopTelemetry = telemetry;
        try {
            feedStop = hw.get(ServoImplEx.class, "FeedStop");
            feedStopReady = true;
            applyFeedStopConfigValues();
            feedStopState = FeedStopState.UNKNOWN;
            homeState = HomeState.IDLE;
            feedStopHomeQueued = true;
            feedStopHomed = false;
            if (this.feedStopTelemetry != null) {
                this.feedStopTelemetry.addLine("FeedStop: homing queued for START");
            }
        } catch (Exception ex) {
            feedStopReady = false;
            feedStopHomed = false;
            if (this.feedStopTelemetry != null) {
                this.feedStopTelemetry.addLine("FeedStop init failed: " + ex.getMessage());
            }
        }
    }

    /** Immediately commands the servo to the BLOCK position. */
    public void setBlock() {
        setHold();
    }

    /** Immediately commands the servo to the HOLD angle (blocking state). */
    public void setHold() {
        releaseUntilMs = 0L;
        feedAllowedAfterMs = 0L;
        if (!feedStopReady || feedStop == null) {
            feedStopState = FeedStopState.UNKNOWN;
            return;
        }
        commandAngleImmediate(holdAngleDeg);
        feedStopState = FeedStopState.BLOCK;
    }

    /** Begin FeedStop homing after START so INIT stays motionless. Safe to call repeatedly. */
    public void startFeedStopAfterStart() {
        if (!feedStopReady || feedStop == null) {
            feedStopHomeQueued = false;
            return;
        }
        if (!feedStopHomeQueued && feedStopHomed) {
            return;
        }
        applyFeedStopConfigValues();
        beginFeedStopHoming();
        feedStopHomeQueued = false;
        if (this.feedStopTelemetry != null) {
            this.feedStopTelemetry.addLine("FeedStop: homing…");
        }
    }

    /** Immediately commands the servo to the homed zero position. */
    public void setHome() {
        releaseUntilMs = 0L;
        feedAllowedAfterMs = 0L;
        if (!feedStopReady || feedStop == null) {
            feedStopState = FeedStopState.UNKNOWN;
            return;
        }
        commandAngleImmediate(softCcwLimitDeg);
        feedStopState = FeedStopState.BLOCK;
    }

    /** Immediately commands the servo to the RELEASE position. */
    public void setRelease() {
        if (!feedStopReady || feedStop == null) {
            feedStopState = FeedStopState.UNKNOWN;
            return;
        }
        commandAngleImmediate(releaseAngleDeg);
        feedStopState = FeedStopState.RELEASE;
    }

    /** Request a release window, extending the hold timer and enforcing fire lead time. */
    public void requestReleaseHold() {
        if (!feedStopReady || feedStop == null) return;
        long now = System.currentTimeMillis();
        releaseUntilMs = now + Math.max(0L, releaseHoldMs);
        setRelease();
    }

    /** Update servo state machine; call every loop. */
    public void update() {
        long now = System.currentTimeMillis();
        updateHoming(now);

        if (continuousFeedActive) {
            applySafetyConfig();
            if (feedStopReady && feedStop != null) {
                commandAngleImmediate(releaseAngleDeg);
                feedStopState = FeedStopState.RELEASE;
            }
            releaseUntilMs = 0L;
            feedAllowedAfterMs = now;
            motor.setPower(firePower);
            return;
        }

        updateFeedStop(now);
        updateFeedCycle(now);
    }

    /** Current servo position (scaled range) or NaN if unavailable. */
    public double getCurrentPosition() {
        if (!feedStopReady || feedStop == null) return Double.NaN;
        return feedStop.getPosition();
    }

    /** Remaining hold time before returning to BLOCK (ms). */
    public long getHoldRemainingMs() {
        if (!feedStopReady || feedStop == null || releaseUntilMs <= 0) return 0L;
        long remaining = releaseUntilMs - System.currentTimeMillis();
        return Math.max(0L, remaining);
    }

    /** Single-line summary for telemetry consumers. */
    public String getFeedStopSummaryLine() {
        String homeStatus = feedStopHomed ? "HOMED" : getFeedStopHomeState();
        String mode = useAutoScale ? "autoScale" : "fullSpan";
        double window = Double.isFinite(windowDegrees) ? windowDegrees : 0.0;
        long holdMs = getHoldRemainingMs();
        return String.format(Locale.US,
                "FeedStop: %s home=%s hold=%.0f° rel=%.0f° mode=%s win=%.0f° limits=[%.0f°,%.0f°] holdMs=%d",
                feedStopState.name(),
                homeStatus,
                holdAngleDeg,
                releaseAngleDeg,
                mode,
                window,
                softCcwLimitDeg,
                softCwLimitDeg,
                holdMs);
    }

    /** Returns the configured fire lead time (ms). */
    public long getFireLeadMs() {
        return Math.max(0L, fireLeadMs);
    }

    /** Returns the configured duration to remain at RELEASE after a feed request (ms). */
    public long getReleaseHoldMs() {
        return Math.max(0L, releaseHoldMs);
    }

    /** Current FeedStop gate state. */
    public FeedStopState getFeedStopState() {
        return feedStopState;
    }

    /** True once the FeedStop homing routine has established zero. */
    public boolean isFeedStopHomed() {
        return feedStopReady && feedStopHomed;
    }

    /** Current homing state name for telemetry. */
    public String getFeedStopHomeState() {
        return homeState.name();
    }

    /** Zero (BLOCK) servo position after homing, or NaN if not homed. */
    public double getZeroPosition() {
        if (!feedStopHomed) return Double.NaN;
        return zeroPosition;
    }

    /** Hold angle in degrees relative to zero. */
    public double getHoldAngleDeg() {
        return holdAngleDeg;
    }

    /** Release angle in degrees relative to zero. */
    public double getReleaseAngleDeg() {
        return releaseAngleDeg;
    }

    /** Total degrees covered by the current scaled PWM window. */
    public double getWindowDegrees() {
        return windowDegrees;
    }

    /** Servo scale lower bound applied via scaleRange(). */
    public double getScaleMin() {
        return appliedScaleMin;
    }

    /** Servo scale upper bound applied via scaleRange(). */
    public double getScaleMax() {
        return appliedScaleMax;
    }

    public double getSoftCcwLimitDeg() {
        return softCcwLimitDeg;
    }

    public double getSoftCwLimitDeg() {
        return softCwLimitDeg;
    }

    /** Direction sign (+1 release toward max, -1 toward min). */
    public double getDirectionSign() {
        return directionSign;
    }

    /** Span required by requested hold/release angles plus safety margin. */
    public double getRequiredSpanDeg() {
        return requiredSpanDeg;
    }

    /** Configured safety margin in degrees. */
    public double getSafetyMarginDeg() {
        return safetyMarginDeg;
    }

    /** True if the scale window was expanded beyond the config values. */
    public boolean wasScaleAdjusted() {
        return autoScaleApplied;
    }

    public boolean isAutoScaleEnabled() {
        return useAutoScale;
    }

    /** True if requested angles were clamped to fit the available window. */
    public boolean wasAngleClamped() {
        return angleClamped;
    }

    /** True if the servo bounds prevented reaching the requested span. */
    public boolean wasWindowLimitReached() {
        return windowLimitReached;
    }

    public boolean wasSoftLimitClamped() {
        return softLimitClamped;
    }

    public String getSoftLimitMessage() {
        return softLimitMessage;
    }

    public boolean wasHomeAborted() {
        return homeAborted;
    }

    public String getHomeAbortMessage() {
        return homeAbortMessage;
    }

    public double getSafePresetOpenDeg() {
        return safePresetOpenDeg;
    }

    public double getMaxHomeTravelDeg() {
        return maxHomeTravelDeg;
    }

    public enum FeedStopState {
        BLOCK,
        RELEASE,
        UNKNOWN
    }

    /** Returns true if enough time has passed since last feed to fire again. */
    public boolean canFire() {
        return System.currentTimeMillis() - lastFire >= minCycleMs;
    }

    /** Timed feed (blocking for ~fireTimeMs). */
    public void feedOnceBlocking() {
        if (!beginFeedCycle()) return;
        while (isFeedCycleActive()) {
            update();
            sleep(10);
        }
        update();
    }

    /** Begin a non-blocking feed cycle that respects FeedStop release/hold timing. */
    public boolean beginFeedCycle() {
        if (cycleState != FeedCycleState.IDLE) return false;
        if (continuousFeedActive) return false;
        if (!canFire()) return false;
        lastFire = System.currentTimeMillis();
        feedAllowedAfterMs = lastFire + Math.max(0L, fireLeadMs);
        cycleState = FeedCycleState.WAIT_FOR_LEAD;
        cycleStateStartMs = lastFire;
        applySafetyConfig();
        setRelease();
        return true;
    }

    /** Returns true while an asynchronous feed cycle is waiting or firing. */
    public boolean isFeedCycleActive() {
        return cycleState != FeedCycleState.IDLE;
    }

    /** Run the feed motor and hold the gate open continuously (used for fire-button holds). */
    public void startContinuousFeed() {
        if (continuousFeedActive) return;
        continuousFeedActive = true;
        lastFire = System.currentTimeMillis();
        feedAllowedAfterMs = lastFire;
        releaseUntilMs = 0L;
        cycleState = FeedCycleState.IDLE;
        applySafetyConfig();
        setRelease();
        motor.setPower(firePower);
    }

    /** Allow external gating so the feed motor waits until ready before firing. */
    public void setFeedReady(boolean ready) {
        feedReadyGate = ready;
    }

    /** Stop continuous-feed mode and return the gate to HOLD. */
    public void stopContinuousFeed() {
        if (!continuousFeedActive) return;
        continuousFeedActive = false;
        applyIdleHoldPower();
        setHold();
        cycleState = FeedCycleState.IDLE;
    }

    /** True when the continuous-feed stream is active. */
    public boolean isContinuousFeedActive() {
        return continuousFeedActive;
    }

    /** Immediately stops the feed motor. */
    public void stop() {
        applySafetyConfig();
        continuousFeedActive = false;
        applyIdleHoldPower();
        setHome();
        cycleState = FeedCycleState.IDLE;
    }

    /** Force BRAKE zero-power behavior with no idle counter-rotation. */
    public void applyBrakeHold() {
        idleHoldActive = false;
        applySafetyConfig();
        continuousFeedActive = false;
        motor.setPower(0.0);
        setHome();
        cycleState = FeedCycleState.IDLE;
    }

    /** Helper exposed for safety fallbacks and testing. */
    public void setPower(double p) {
        applySafetyConfig();
        motor.setPower(p);
    }

    private void sleep(int ms) {
        try { Thread.sleep(ms); } catch (InterruptedException ignored) {}
    }

    private void applySafetyConfig() {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (motor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private void applyIdleHoldPower() {
        double power = (idleHoldActive) ? idleHoldPower : 0.0;
        if (Math.abs(power) < 1e-6) power = 0.0;
        motor.setPower(power);
    }

    private void updateFeedStop(long now) {
        if (!feedStopReady || feedStop == null) return;
        if (feedStopState == FeedStopState.RELEASE && releaseUntilMs > 0) {
            if (now >= releaseUntilMs) {
                setHold();
            }
        }
    }

    private void updateFeedCycle(long now) {
        switch (cycleState) {
            case IDLE:
                return;
            case WAIT_FOR_LEAD:
                if (now >= feedAllowedAfterMs && feedReadyGate) {
                    requestReleaseHold();
                    motor.setPower(firePower);
                    cycleState = FeedCycleState.FEEDING;
                    cycleStateStartMs = now;
                }
                break;
            case FEEDING:
                if (now - cycleStateStartMs >= Math.max(0, fireTimeMs)) {
                    applyIdleHoldPower();
                    cycleState = FeedCycleState.IDLE;
                }
                break;
        }
    }

    private void updateHoming(long now) {
        if (!feedStopReady || feedStop == null) return;
        switch (homeState) {
            case SAFE_OPEN_INIT:
                homeCommandedDeg = softCcwLimitDeg;
                homeTargetDeg = clampAngleForConfig(Math.max(safePresetOpenDeg, softCcwLimitDeg));
                homeTravelAccumDeg = 0.0;
                lastHomeStepMs = 0L;
                if (homeTargetDeg <= softCcwLimitDeg + 1e-6) {
                    homeState = HomeState.MOVE_TO_ZERO;
                } else {
                    homeState = HomeState.SAFE_OPEN_STEP;
                }
                homeStateStartMs = now;
                break;
            case SAFE_OPEN_STEP:
                if (homeAborted) {
                    homeState = HomeState.ABORTED;
                    homeStateStartMs = now;
                    break;
                }
                if (homeTargetDeg <= softCcwLimitDeg + 1e-6) {
                    homeState = HomeState.MOVE_TO_ZERO;
                    homeStateStartMs = now;
                    break;
                }
                if (lastHomeStepMs == 0L || (now - lastHomeStepMs) >= HOME_STEP_INTERVAL_MS) {
                    double nextDeg = Math.min(homeTargetDeg, homeCommandedDeg + HOME_STEP_DEG);
                    double delta = Math.max(0.0, nextDeg - homeCommandedDeg);
                    if (homeTravelAccumDeg + delta > maxHomeTravelDeg + 1e-6) {
                        abortHoming("Home approach exceeded safe travel; reposition gate near top and re-init.", now);
                        break;
                    }
                    commandAngleImmediate(nextDeg);
                    homeTravelAccumDeg += delta;
                    homeCommandedDeg = nextDeg;
                    lastHomeStepMs = now;
                    if (Math.abs(homeTargetDeg - homeCommandedDeg) <= 1e-6) {
                        homeState = HomeState.MOVE_TO_ZERO;
                        homeStateStartMs = now;
                    }
                }
                break;
            case MOVE_TO_ZERO:
                commandAngleImmediate(softCcwLimitDeg);
                homeState = HomeState.DWELL_AT_ZERO;
                homeStateStartMs = now;
                break;
            case DWELL_AT_ZERO:
                if ((now - homeStateStartMs) >= homeDwellMs) {
                    homeState = HomeState.BACKOFF_FROM_ZERO;
                    homeStateStartMs = now;
                }
                break;
            case BACKOFF_FROM_ZERO:
                finalizeHome(now);
                homeState = HomeState.COMPLETE;
                homeStateStartMs = now;
                break;
            case ABORTED:
                if (feedStopTelemetry != null && homeAbortMessage != null && !scaleTelemetryEmitted) {
                    feedStopTelemetry.addLine("FeedStop: " + homeAbortMessage);
                    scaleTelemetryEmitted = true;
                }
                break;
            case COMPLETE:
            case IDLE:
            default:
                break;
        }
    }

    private void beginFeedStopHoming() {
        if (!feedStopReady || feedStop == null) return;
        feedStopHomed = false;
        homeAborted = false;
        homeAbortMessage = null;
        releaseUntilMs = 0L;
        feedAllowedAfterMs = 0L;
        feedStopState = FeedStopState.UNKNOWN;
        homeState = FeedStopConfig.SKIP_SAFE_OPEN_ON_START ? HomeState.MOVE_TO_ZERO : HomeState.SAFE_OPEN_INIT;
        homeStateStartMs = 0L;
        scaleTelemetryEmitted = false;
    }

    private void applyFeedStopConfigValues() {
        directionSign = resolveDirectionSign(FeedStopConfig.DIRECTION_SIGN);
        useAutoScale = FeedStopConfig.USE_AUTO_SCALE;
        double configuredSafeOpen = Math.max(0.0, FeedStopConfig.SAFE_PRESET_OPEN_DEG);
        double legacyOvershoot = Math.max(0.0, FeedStopConfig.HOME_OVERSHOOT_DEG);
        safePresetOpenDeg = Math.max(configuredSafeOpen, legacyOvershoot);
        homeBackoffDeg = Math.max(0.0, FeedStopConfig.HOME_BACKOFF_DEG);
        homeDwellMs = Math.max(0L, FeedStopConfig.HOME_DWELL_MS);
        softCcwLimitDeg = Math.max(0.0, FeedStopConfig.SOFT_CCW_LIMIT_DEG);
        softCwLimitDeg = Math.max(softCcwLimitDeg, FeedStopConfig.SOFT_CW_LIMIT_DEG);
        maxHomeTravelDeg = Math.max(softCwLimitDeg, FeedStopConfig.MAX_HOME_TRAVEL_DEG);
        safetyMarginDeg = Math.max(0.0, FeedStopConfig.SAFETY_MARGIN_DEG);
        setTunables(FeedStopConfig.HOLD_ANGLE_DEG,
                FeedStopConfig.RELEASE_ANGLE_DEG,
                FeedStopConfig.RELEASE_HOLD_MS,
                FeedStopConfig.FIRE_LEAD_MS);
    }

    public void setTunables(double holdAngleDeg, double releaseAngleDeg, long holdMs, long leadMs) {
        configuredHoldAngleDeg = holdAngleDeg;
        configuredReleaseAngleDeg = releaseAngleDeg;
        releaseHoldMs = Math.max(0L, holdMs);
        fireLeadMs = Math.max(0L, leadMs);
        recalcScaleAndAngles();
        recomputeTargetPositions();
        if (feedStopReady && feedStop != null) {
            applyScaleRangeSettings();
            if (feedStopState == FeedStopState.RELEASE) {
                commandAngleImmediate(releaseAngleDeg);
            } else if (feedStopState == FeedStopState.BLOCK) {
                commandAngleImmediate(holdAngleDeg);
            }
        }
    }

    private void recalcScaleAndAngles() {
        requiredSpanDeg = Math.max(
                Math.abs(configuredHoldAngleDeg),
                Math.abs(configuredReleaseAngleDeg)) + safetyMarginDeg;
        requiredSpanDeg = Math.max(0.0, requiredSpanDeg);

        windowLimitReached = false;
        angleClamped = false;

        double zero = resolvedZeroPosition();
        if (!Double.isFinite(zero)) {
            zero = blockHardstopPosition();
        }
        zero = clip(zero);

        double softSpan = Math.max(0.0, softCwLimitDeg - softCcwLimitDeg);
        double targetSpanDeg = Math.max(requiredSpanDeg, softSpan);

        if (useAutoScale) {
            double targetSpanNorm = Math.min(1.0, Math.max(0.0, targetSpanDeg / FULL_SPAN_DEG));
            double halfSpan = targetSpanNorm / 2.0;

            double min = zero - halfSpan;
            double max = zero + halfSpan;

            if (min < 0.0) {
                max -= min;
                min = 0.0;
            }
            if (max > 1.0) {
                double over = max - 1.0;
                min -= over;
                max = 1.0;
            }

            min = Math.max(0.0, min);
            max = Math.min(1.0, max);

            double achievedSpan = Math.max(0.0, max - min);
            if (achievedSpan + 1e-6 < targetSpanNorm) {
                windowLimitReached = true;
            }

            appliedScaleMin = min;
            appliedScaleMax = max;
            windowDegrees = Math.max(1e-3, achievedSpan * FULL_SPAN_DEG);
            autoScaleApplied = achievedSpan > 1e-6;
        } else {
            appliedScaleMin = Double.NaN;
            appliedScaleMax = Double.NaN;
            windowDegrees = FULL_SPAN_DEG;
            autoScaleApplied = false;
        }

        double availableWindow = Math.max(0.0, windowDegrees - safetyMarginDeg);
        maxAvailableAngleDeg = Math.max(0.0, Math.min(availableWindow, softSpan));

        double releaseAfterSoft = clampAngleForConfig(configuredReleaseAngleDeg);
        double holdAfterSoft = clampAngleForConfig(configuredHoldAngleDeg);

        double limitedRelease = clampToAvailable(releaseAfterSoft);
        double limitedHold = clampToAvailable(holdAfterSoft);

        if (Math.abs(limitedHold) > Math.abs(limitedRelease) + 1e-6) {
            angleClamped = true;
            limitedHold = Math.copySign(Math.abs(limitedRelease), limitedHold == 0.0 ? 1.0 : limitedHold);
        }

        releaseAngleDeg = limitedRelease;
        holdAngleDeg = limitedHold;

        scaleTelemetryEmitted = false;
    }

    private void recomputeTargetPositions() {
        double zero = resolvedZeroPosition();
        holdPosition = clip(zero + degreesToUnits(holdAngleDeg));
        releasePosition = clip(zero + degreesToUnits(releaseAngleDeg));
    }

    private double resolvedZeroPosition() {
        if (feedStopHomed && !Double.isNaN(zeroPosition)) {
            return clip(zeroPosition);
        }
        return clip(blockHardstopPosition());
    }

    private double resolvedHoldPosition() {
        if (!Double.isNaN(holdPosition)) {
            return clip(holdPosition);
        }
        return clip(resolvedZeroPosition() + degreesToUnits(holdAngleDeg));
    }

    private double resolvedReleasePosition() {
        if (!Double.isNaN(releasePosition)) {
            return clip(releasePosition);
        }
        return clip(resolvedZeroPosition() + degreesToUnits(releaseAngleDeg));
    }

    private double blockHardstopPosition() {
        return (directionSign >= 0.0) ? 0.0 : 1.0;
    }

    private double degreesToUnits(double degrees) {
        if (!Double.isFinite(degrees) || windowDegrees <= 1e-6) return 0.0;
        double units = degrees / windowDegrees;
        if (!Double.isFinite(units)) return 0.0;
        return units * directionSign;
    }

    private double commandAngleImmediate(double targetDeg) {
        if (!feedStopReady || feedStop == null) return clampAngleForCommand(targetDeg);
        double commanded = clampAngleForCommand(targetDeg);
        double position = positionForAngle(commanded);
        feedStop.setPosition(position);
        return commanded;
    }

    private double positionForAngle(double angleDeg) {
        double zero = resolvedZeroPosition();
        return clip(zero + degreesToUnits(angleDeg));
    }

    private double clampAngleForCommand(double requested) {
        double afterSoft = clampAngleForConfig(requested);
        return clampToAvailable(afterSoft);
    }

    private double clampAngleForConfig(double requested) {
        if (!Double.isFinite(requested)) {
            requested = softCcwLimitDeg;
        }
        double clamped = requested;
        if (clamped < softCcwLimitDeg) {
            noteSoftLimitClamp(requested, softCcwLimitDeg);
            clamped = softCcwLimitDeg;
        }
        if (clamped > softCwLimitDeg) {
            noteSoftLimitClamp(requested, softCwLimitDeg);
            clamped = softCwLimitDeg;
        }
        return clamped;
    }

    private double clampToAvailable(double requested) {
        double sign = Math.signum(requested == 0.0 ? 1.0 : requested);
        double magnitude = Math.abs(requested);
        double maxMagnitude = Math.max(0.0, maxAvailableAngleDeg);
        if (magnitude > maxMagnitude + 1e-6) {
            angleClamped = true;
            magnitude = maxMagnitude;
        }
        return magnitude * sign;
    }

    private void noteSoftLimitClamp(double requested, double clamped) {
        if (softLimitClamped || Math.abs(clamped - requested) <= 1e-6) {
            return;
        }
        softLimitClamped = true;
        softLimitMessage = String.format(Locale.US,
                "FeedStop: requests clamped to soft limits [%.0f°, %.0f°].",
                softCcwLimitDeg, softCwLimitDeg);
    }

    private void applyScaleRangeSettings() {
        if (!feedStopReady || feedStop == null) return;
        if (useAutoScale && Double.isFinite(appliedScaleMin) && Double.isFinite(appliedScaleMax)) {
            double span = appliedScaleMax - appliedScaleMin;
            if (span > 1e-6) {
                feedStop.scaleRange(appliedScaleMin, appliedScaleMax);
                autoScaleApplied = true;
                return;
            }
        }
        if (autoScaleApplied) {
            feedStop.scaleRange(0.0, 1.0);
        }
        autoScaleApplied = false;
    }

    private void finalizeHome(long now) {
        double zero = clip(blockHardstopPosition());
        zeroPosition = zero;
        feedStopHomed = true;
        recalcScaleAndAngles();
        recomputeTargetPositions();
        applyScaleRangeSettings();

        double backoff = Math.max(softCcwLimitDeg, homeBackoffDeg);
        commandAngleImmediate(backoff);
        feedStopState = FeedStopState.BLOCK;
        setHold();
        maybeEmitScaleTelemetry();
        homeCommandedDeg = holdAngleDeg;
    }

    private void abortHoming(String message, long now) {
        if (homeAborted) return;
        homeAborted = true;
        homeAbortMessage = message;
        homeState = HomeState.ABORTED;
        homeStateStartMs = now;
        if (feedStopTelemetry != null && message != null) {
            feedStopTelemetry.addLine("FeedStop: " + message);
        }
        scaleTelemetryEmitted = true;
    }

    private static double resolveDirectionSign(double raw) {
        if (Double.isNaN(raw) || raw == 0.0) return 1.0;
        return (raw >= 0.0) ? 1.0 : -1.0;
    }

    private double clip(double v) {
        if (Double.isNaN(v)) return 0.0;
        return Math.max(0.0, Math.min(1.0, v));
    }

    private void maybeEmitScaleTelemetry() {
        if (scaleTelemetryEmitted || feedStopTelemetry == null) return;
        scaleTelemetryEmitted = true;
        if (windowLimitReached) {
            feedStopTelemetry.addLine("FeedStop: scale window hit bounds; hold/release clamped to avoid binding.");
        } else if (angleClamped) {
            feedStopTelemetry.addLine("FeedStop: hold/release clamped to fit available window.");
        }
        if (softLimitClamped && softLimitMessage != null) {
            feedStopTelemetry.addLine(softLimitMessage);
        }
    }
}
