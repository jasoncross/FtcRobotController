package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.config.FeedTuning;
import org.firstinspires.ftc.teamcode.config.SharedRobotTuning;
import org.firstinspires.ftc.teamcode.subsystems.Feed;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.Locale;

/*
 * FILE: FiringController.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/control/
 *
 * PURPOSE
 *   - Own the shared firing transaction state machine so TeleOp and Auto use
 *     identical shot sequencing.
 *   - Coordinate FeedStop timing, feed motor power, intake suppression, and
 *     RPM drop detection without blocking the OpMode loop.
 *
 * NOTES
 *   - TeleOp should call update(...) every loop with live launcher/aim data so
 *     the controller can latch shot targets and detect readiness internally.
 *   - Auto helpers may block on isIdle() while calling update(...) inside loops.
 * CHANGES (2026-01-05): Added per-shot readiness snapshots, recovery band exits,
 *                       and feedstop-return fixes for continuous fire.
 * CHANGES (2026-01-06): Keep the FeedStop released throughout continuous/spray
 *                       streams, only returning to HOLD once the stream ends.
 */
public class FiringController {
    private static final int HISTORY_SIZE = 5;

    public enum State {
        IDLE,
        FIRE_REQUESTED,
        AUTOAIM_WINDOW,
        RPM_WINDOW,
        FEEDSTOP_OPEN,
        FEEDING,
        SHOT_DETECTED,
        FEEDSTOP_RETURNS,
        RECOVERING
    }

    public enum Mode {
        SINGLE,
        CONTINUOUS,
        SPRAY
    }

    private final Feed feed;
    private final Intake intake;

    private State state = State.IDLE;
    private Mode mode = Mode.SINGLE;
    private long stateStartMs = 0L;
    private boolean transactionActive = false;
    private final long[] stateTotalsMs = new long[State.values().length];
    private final long[] lastShotTotalsMs = new long[State.values().length];
    private boolean lastShotComplete = false;
    private final long[][] shotHistoryTotalsMs = new long[HISTORY_SIZE][State.values().length];
    private final Mode[] shotHistoryModes = new Mode[HISTORY_SIZE];
    private final double[] shotHistoryTargets = new double[HISTORY_SIZE];
    private int shotHistoryCount = 0;
    private int shotHistoryIndex = 0;

    private boolean continuousRequested = false;
    private boolean sprayLike = false;
    private boolean firstShotInStream = true;
    private boolean preReadyLatchedAtRequest = false;

    private boolean autoAimAllowed = true;
    private long autoAimWindowMs = 0L;
    private long autoAimStartMs = 0L;

    private SharedRobotTuning.HoldFireForRpmMode rpmGateMode = SharedRobotTuning.HoldFireForRpmMode.ALL;

    private boolean intakeSuppressed = false;
    private boolean intakeRestoreState = false;

    private long feedLeadReadyAtMs = 0L;
    private long feedingStartMs = 0L;
    private long feedStopReturnAtMs = 0L;
    private long strictReadyStartMs = 0L;
    private long recoveryHoldStartMs = 0L;

    private boolean feedMotorStarted = false;
    private long feedMotorStartedAtMs = 0L;
    private boolean latchedTargetValid = false;
    private double latchedShotTargetRpm = 0.0;
    private double avgRpm = 0.0;
    private boolean recovered = false;

    public FiringController(Feed feed, Intake intake) {
        this.feed = feed;
        this.intake = intake;
    }

    public void reset() {
        state = State.IDLE;
        mode = Mode.SINGLE;
        stateStartMs = -1L;
        transactionActive = false;
        clearTimingTotals();
        clearLastShotTotals();
        clearShotHistory();
        lastShotComplete = false;
        continuousRequested = false;
        sprayLike = false;
        firstShotInStream = true;
        preReadyLatchedAtRequest = false;
        autoAimAllowed = true;
        autoAimWindowMs = 0L;
        autoAimStartMs = 0L;
        rpmGateMode = SharedRobotTuning.HoldFireForRpmMode.ALL;
        intakeSuppressed = false;
        intakeRestoreState = false;
        feedLeadReadyAtMs = 0L;
        feedingStartMs = 0L;
        feedStopReturnAtMs = 0L;
        strictReadyStartMs = 0L;
        recoveryHoldStartMs = 0L;
        feedMotorStarted = false;
        feedMotorStartedAtMs = 0L;
        latchedTargetValid = false;
        latchedShotTargetRpm = 0.0;
        avgRpm = 0.0;
        recovered = false;
        if (feed != null) {
            feed.setPower(0.0);
            feed.setHold();
        }
        restoreIntake();
    }

    public State getState() {
        return state;
    }

    public Mode getMode() {
        return mode;
    }

    public boolean isIdle() {
        return state == State.IDLE;
    }

    public boolean isAutoAimWindowActive() {
        return state == State.AUTOAIM_WINDOW;
    }

    public boolean isFeedActive() {
        return state == State.FEEDING
                || state == State.SHOT_DETECTED
                || state == State.RECOVERING;
    }

    public boolean isSprayActive() {
        return mode == Mode.SPRAY && continuousRequested;
    }

    public String getTelemetrySummary() {
        return mode.name() + " / " + state.name();
    }

    public boolean consumeFeedMotorStarted() {
        boolean started = feedMotorStarted;
        feedMotorStarted = false;
        return started;
    }

    public double getLatchedShotTargetRpm() {
        return latchedShotTargetRpm;
    }

    public boolean wasPreReadyLatchedAtRequest() {
        return preReadyLatchedAtRequest;
    }

    public double getAvgRpm() {
        return avgRpm;
    }

    public double getErrorRpm() {
        if (!latchedTargetValid || !Double.isFinite(latchedShotTargetRpm)) {
            return 0.0;
        }
        return latchedShotTargetRpm - avgRpm;
    }

    public boolean isRecovered() {
        return recovered;
    }

    public String getLastShotTimingSummary() {
        return String.format(Locale.US, "t(ms) FR=%d AIM=%d RPM=%d OPEN=%d FEED=%d SHOT=%d RET=%d REC=%d",
                lastShotTotalsMs[State.FIRE_REQUESTED.ordinal()],
                lastShotTotalsMs[State.AUTOAIM_WINDOW.ordinal()],
                lastShotTotalsMs[State.RPM_WINDOW.ordinal()],
                lastShotTotalsMs[State.FEEDSTOP_OPEN.ordinal()],
                lastShotTotalsMs[State.FEEDING.ordinal()],
                lastShotTotalsMs[State.SHOT_DETECTED.ordinal()],
                lastShotTotalsMs[State.FEEDSTOP_RETURNS.ordinal()],
                lastShotTotalsMs[State.RECOVERING.ordinal()]);
    }

    public String getShotHistorySummary() {
        if (shotHistoryCount <= 0) {
            return "Hist: -";
        }
        StringBuilder summary = new StringBuilder("Hist:");
        for (int i = 0; i < shotHistoryCount; i++) {
            int idx = (shotHistoryIndex - 1 - i + HISTORY_SIZE) % HISTORY_SIZE;
            Mode shotMode = shotHistoryModes[idx];
            double target = shotHistoryTargets[idx];
            summary.append(" [")
                    .append(shotMode != null ? shotMode.name().charAt(0) : '?')
                    .append(" ")
                    .append((target > 0.0 && Double.isFinite(target)) ? String.format(Locale.US, "%.0f", target) : "N/A")
                    .append(" FR=").append(shotHistoryTotalsMs[idx][State.FIRE_REQUESTED.ordinal()])
                    .append(" AIM=").append(shotHistoryTotalsMs[idx][State.AUTOAIM_WINDOW.ordinal()])
                    .append(" RPM=").append(shotHistoryTotalsMs[idx][State.RPM_WINDOW.ordinal()])
                    .append(" OPEN=").append(shotHistoryTotalsMs[idx][State.FEEDSTOP_OPEN.ordinal()])
                    .append(" FEED=").append(shotHistoryTotalsMs[idx][State.FEEDING.ordinal()])
                    .append(" SHOT=").append(shotHistoryTotalsMs[idx][State.SHOT_DETECTED.ordinal()])
                    .append(" RET=").append(shotHistoryTotalsMs[idx][State.FEEDSTOP_RETURNS.ordinal()])
                    .append(" REC=").append(shotHistoryTotalsMs[idx][State.RECOVERING.ordinal()])
                    .append("]");
        }
        return summary.toString();
    }

    public void setIntakeDesiredState(boolean intakeDesiredOn) {
        intakeRestoreState = intakeDesiredOn;
        if (state == State.IDLE && !intakeSuppressed) {
            restoreIntake();
        }
    }

    public long getFeedMotorStartedAtMs() {
        return feedMotorStartedAtMs;
    }

    public void requestFire(long nowMs,
                            boolean continuousRequested,
                            boolean sprayLike,
                            boolean preReadyLatchedAtRequest,
                            boolean intakeDesiredOn,
                            SharedRobotTuning.HoldFireForRpmMode rpmGateMode,
                            boolean autoAimAllowed,
                            long autoAimWindowMs) {
        if (state != State.IDLE) {
            return;
        }
        this.continuousRequested = continuousRequested;
        this.sprayLike = sprayLike;
        this.autoAimAllowed = autoAimAllowed;
        this.autoAimWindowMs = Math.max(0L, autoAimWindowMs);
        this.rpmGateMode = (rpmGateMode != null) ? rpmGateMode : SharedRobotTuning.HoldFireForRpmMode.ALL;
        this.firstShotInStream = true;
        this.feedMotorStarted = false;
        this.feedMotorStartedAtMs = 0L;
        this.strictReadyStartMs = 0L;
        this.recoveryHoldStartMs = 0L;
        latchedTargetValid = false;
        latchedShotTargetRpm = 0.0;
        clearLastShotTotals();
        beginTransaction();
        this.preReadyLatchedAtRequest = preReadyLatchedAtRequest;
        updateMode();
        intakeRestoreState = intakeDesiredOn;
        if (intake != null && !intakeDesiredOn) {
            intake.set(true);
        }
        transition(State.FIRE_REQUESTED, nowMs);
    }

    public void setContinuousRequested(boolean requested) {
        this.continuousRequested = requested;
        updateMode();
    }

    public void setSprayLike(boolean sprayLike) {
        this.sprayLike = sprayLike;
        updateMode();
    }

    public void update(long nowMs,
                       boolean aimReady,
                       double targetRpm,
                       double leftRpm,
                       double rightRpm,
                       boolean intakeDesiredOn) {
        intakeRestoreState = intakeDesiredOn;
        avgRpm = (leftRpm + rightRpm) / 2.0;
        if (state == State.FIRE_REQUESTED && !latchedTargetValid) {
            latchTargetRpm(targetRpm);
        }
        recovered = isRecoverySatisfied(nowMs);

        switch (state) {
            case IDLE:
                applyIdleHold();
                if (intakeSuppressed) {
                    restoreIntake();
                }
                break;
            case FIRE_REQUESTED:
                ensureIntakeOn();
                if (shouldRunAutoAimWindow()) {
                    autoAimStartMs = nowMs;
                    transition(State.AUTOAIM_WINDOW, nowMs);
                } else if (shouldSkipRpmWindow()) {
                    preReadyLatchedAtRequest = false;
                    transition(State.FEEDSTOP_OPEN, nowMs);
                } else if (preReadyLatchedAtRequest) {
                    preReadyLatchedAtRequest = false;
                    transition(State.FEEDSTOP_OPEN, nowMs);
                } else {
                    transition(State.RPM_WINDOW, nowMs);
                }
                break;
            case AUTOAIM_WINDOW:
                ensureIntakeOn();
                if (aimReady || isAutoAimTimeout(nowMs)) {
                    if (shouldSkipRpmWindow()) {
                        preReadyLatchedAtRequest = false;
                        transition(State.FEEDSTOP_OPEN, nowMs);
                    } else if (preReadyLatchedAtRequest) {
                        preReadyLatchedAtRequest = false;
                        transition(State.FEEDSTOP_OPEN, nowMs);
                    } else {
                        transition(State.RPM_WINDOW, nowMs);
                    }
                }
                break;
            case RPM_WINDOW:
                ensureIntakeOn();
                if (shouldSkipRpmWindow()) {
                    preReadyLatchedAtRequest = false;
                    transition(State.FEEDSTOP_OPEN, nowMs);
                } else if (preReadyLatchedAtRequest) {
                    preReadyLatchedAtRequest = false;
                    transition(State.FEEDSTOP_OPEN, nowMs);
                } else if (isStrictReady(nowMs)) {
                    transition(State.FEEDSTOP_OPEN, nowMs);
                }
                break;
            case FEEDSTOP_OPEN:
                ensureIntakeOn();
                if (feed != null) {
                    feed.setRelease();
                }
                if (feedLeadReadyAtMs == 0L) {
                    feedLeadReadyAtMs = nowMs + Math.max(0L, feed != null ? feed.getFireLeadMs() : 0L);
                }
                if (nowMs >= feedLeadReadyAtMs) {
                    transition(State.FEEDING, nowMs);
                }
                break;
            case FEEDING:
                ensureIntakeOn();
                if (feed != null) {
                    feed.setPower(FeedTuning.FIRE_POWER);
                }
                if (feedingStartMs == 0L) {
                    feedingStartMs = nowMs;
                }
                if (!feedMotorStarted) {
                    feedMotorStarted = true;
                    feedMotorStartedAtMs = nowMs;
                }
                if (shotDetected(targetRpm, leftRpm, rightRpm)
                        || (nowMs - feedingStartMs) >= Math.max(0L, FeedTuning.FIRE_TIME_MS)) {
                    transition(State.SHOT_DETECTED, nowMs);
                }
                break;
            case SHOT_DETECTED:
                firstShotInStream = false;
                if (feed != null) {
                    feed.setPower(FeedTuning.FIRE_POWER_LAUNCHING);
                }
                suppressIntake();
                if (shouldKeepFeedStopOpen()) {
                    transition(State.RECOVERING, nowMs);
                } else {
                    feedStopReturnAtMs = nowMs + Math.max(0L, feed != null ? feed.getReleaseHoldMs() : 0L);
                    transition(State.FEEDSTOP_RETURNS, nowMs);
                }
                break;
            case FEEDSTOP_RETURNS:
                suppressIntake();
                if (nowMs >= feedStopReturnAtMs) {
                    if (feed != null && !shouldKeepFeedStopOpen()) {
                        feed.setHold();
                    }
                    transition(State.RECOVERING, nowMs);
                }
                break;
            case RECOVERING:
                if (feed != null) {
                    feed.setPower(FeedTuning.FIRE_POWER_LAUNCHING);
                }
                suppressIntake();
                if (recovered || isRecoveryTimeout(nowMs)) {
                    finishTransaction(nowMs);
                    restoreIntake();
                    if (continuousRequested) {
                        boolean readyNow = isStrictReady(nowMs);
                        latchedTargetValid = false;
                        latchedShotTargetRpm = 0.0;
                        beginTransaction();
                        preReadyLatchedAtRequest = readyNow;
                        transition(State.FIRE_REQUESTED, nowMs);
                    } else {
                        if (feed != null && feed.getFeedStopState() == Feed.FeedStopState.RELEASE) {
                            feed.setHold();
                        }
                        transition(State.IDLE, nowMs);
                    }
                }
                break;
            default:
                break;
        }
    }

    private void transition(State next, long nowMs) {
        recordStateElapsed(nowMs);
        state = next;
        stateStartMs = nowMs;
        if (next == State.FIRE_REQUESTED) {
            feedMotorStarted = false;
            feedMotorStartedAtMs = 0L;
        }
        if (next != State.FEEDING) {
            feedingStartMs = 0L;
        }
        if (next != State.FEEDSTOP_RETURNS) {
            feedStopReturnAtMs = 0L;
        }
        if (next != State.FEEDSTOP_OPEN) {
            feedLeadReadyAtMs = 0L;
        }
        if (next != State.RPM_WINDOW) {
            strictReadyStartMs = 0L;
        }
        if (next != State.RECOVERING) {
            recoveryHoldStartMs = 0L;
        }
    }

    private void updateMode() {
        if (sprayLike) {
            mode = Mode.SPRAY;
        } else if (continuousRequested) {
            mode = Mode.CONTINUOUS;
        } else {
            mode = Mode.SINGLE;
        }
    }

    private boolean shouldRunAutoAimWindow() {
        return autoAimAllowed
                && autoAimWindowMs > 0L
                && !(continuousRequested && !firstShotInStream);
    }

    private boolean shouldKeepFeedStopOpen() {
        return continuousRequested || sprayLike;
    }

    private boolean isAutoAimTimeout(long nowMs) {
        if (autoAimWindowMs <= 0L) {
            return true;
        }
        return (nowMs - autoAimStartMs) >= autoAimWindowMs;
    }

    private boolean shouldHoldForRpm() {
        if (sprayLike) {
            return false;
        }
        if (rpmGateMode == null) {
            return true;
        }
        switch (rpmGateMode) {
            case OFF:
                return false;
            case INITIAL:
                return firstShotInStream;
            case ALL:
            default:
                return true;
        }
    }

    private boolean shouldSkipRpmWindow() {
        return sprayLike || !shouldHoldForRpm();
    }

    private boolean shotDetected(double targetRpm, double leftRpm, double rightRpm) {
        double referenceTarget = latchedTargetValid ? latchedShotTargetRpm : targetRpm;
        if (referenceTarget <= 0.0 || !Double.isFinite(referenceTarget)) {
            return false;
        }
        double threshold = Math.max(0.0, FeedTuning.FIRING_DROP_RPM_THRESHOLD);
        return (referenceTarget - leftRpm) >= threshold
                || (referenceTarget - rightRpm) >= threshold;
    }

    private void latchTargetRpm(double targetRpm) {
        if (targetRpm > 0.0 && Double.isFinite(targetRpm)) {
            latchedShotTargetRpm = targetRpm;
            latchedTargetValid = true;
        } else {
            latchedShotTargetRpm = 0.0;
            latchedTargetValid = false;
        }
        recovered = false;
    }

    private boolean isStrictReady(long nowMs) {
        if (!latchedTargetValid || latchedShotTargetRpm <= 0.0) {
            strictReadyStartMs = 0L;
            return false;
        }
        double tolerance = Math.max(0.0, SharedRobotTuning.RPM_TOLERANCE);
        double error = Math.abs(avgRpm - latchedShotTargetRpm);
        if (error <= tolerance) {
            if (strictReadyStartMs == 0L) {
                strictReadyStartMs = nowMs;
            }
            long settleMs = Math.max(0L, SharedRobotTuning.RPM_READY_SETTLE_MS);
            return (nowMs - strictReadyStartMs) >= settleMs;
        }
        strictReadyStartMs = 0L;
        return false;
    }

    private boolean isRecoverySatisfied(long nowMs) {
        if (state == State.IDLE) {
            return lastShotComplete;
        }
        if (state != State.RECOVERING) {
            return false;
        }
        if (!latchedTargetValid || latchedShotTargetRpm <= 0.0) {
            return true;
        }
        double band = Math.max(0.0, SharedRobotTuning.FIRING_RECOVERY_RPM_BAND);
        double threshold = latchedShotTargetRpm - band;
        if (avgRpm >= threshold) {
            long holdMs = Math.max(0L, SharedRobotTuning.FIRING_RECOVERY_HOLD_MS);
            if (holdMs <= 0L) {
                return true;
            }
            if (recoveryHoldStartMs == 0L) {
                recoveryHoldStartMs = nowMs;
            }
            return (nowMs - recoveryHoldStartMs) >= holdMs;
        }
        recoveryHoldStartMs = 0L;
        return false;
    }

    private boolean isRecoveryTimeout(long nowMs) {
        long maxMs = Math.max(0L, SharedRobotTuning.FIRING_RECOVERY_MAX_MS);
        return maxMs > 0L && (nowMs - stateStartMs) >= maxMs;
    }

    private void beginTransaction() {
        transactionActive = true;
        clearTimingTotals();
        lastShotComplete = false;
        recovered = false;
    }

    private void finishTransaction(long nowMs) {
        recordStateElapsed(nowMs);
        copyTimingTotals();
        transactionActive = false;
        lastShotComplete = true;
        recovered = true;
        recordShotHistory();
    }

    private void recordStateElapsed(long nowMs) {
        if (!transactionActive || stateStartMs < 0L) {
            return;
        }
        int idx = state.ordinal();
        if (idx >= 0 && idx < stateTotalsMs.length) {
            stateTotalsMs[idx] += Math.max(0L, nowMs - stateStartMs);
        }
    }

    private void clearTimingTotals() {
        for (int i = 0; i < stateTotalsMs.length; i++) {
            stateTotalsMs[i] = 0L;
        }
    }

    private void clearLastShotTotals() {
        for (int i = 0; i < lastShotTotalsMs.length; i++) {
            lastShotTotalsMs[i] = 0L;
        }
    }

    private void clearShotHistory() {
        shotHistoryCount = 0;
        shotHistoryIndex = 0;
        for (int i = 0; i < HISTORY_SIZE; i++) {
            shotHistoryModes[i] = null;
            shotHistoryTargets[i] = 0.0;
            for (int j = 0; j < shotHistoryTotalsMs[i].length; j++) {
                shotHistoryTotalsMs[i][j] = 0L;
            }
        }
    }

    private void copyTimingTotals() {
        for (int i = 0; i < stateTotalsMs.length && i < lastShotTotalsMs.length; i++) {
            lastShotTotalsMs[i] = stateTotalsMs[i];
        }
    }

    private void recordShotHistory() {
        int idx = shotHistoryIndex % HISTORY_SIZE;
        for (int i = 0; i < stateTotalsMs.length && i < shotHistoryTotalsMs[idx].length; i++) {
            shotHistoryTotalsMs[idx][i] = stateTotalsMs[i];
        }
        shotHistoryModes[idx] = mode;
        shotHistoryTargets[idx] = latchedTargetValid ? latchedShotTargetRpm : 0.0;
        shotHistoryIndex = (shotHistoryIndex + 1) % HISTORY_SIZE;
        shotHistoryCount = Math.min(shotHistoryCount + 1, HISTORY_SIZE);
    }

    private void ensureIntakeOn() {
        if (intake == null) {
            return;
        }
        if (!intake.isOn()) {
            intake.set(true);
        }
        intakeSuppressed = false;
    }

    private void suppressIntake() {
        if (intake == null) {
            return;
        }
        if (!intakeSuppressed) {
            intakeSuppressed = true;
        }
        intake.set(false);
    }

    private void restoreIntake() {
        if (intake == null) {
            return;
        }
        intake.set(intakeRestoreState);
        intakeSuppressed = false;
    }

    private void applyIdleHold() {
        if (feed == null) {
            return;
        }
        feed.setPower(FeedTuning.IDLE_HOLD_POWER);
    }
}
