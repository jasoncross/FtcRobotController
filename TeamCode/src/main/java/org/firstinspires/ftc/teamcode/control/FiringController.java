package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.config.FeedTuning;
import org.firstinspires.ftc.teamcode.config.SharedRobotTuning;
import org.firstinspires.ftc.teamcode.subsystems.Feed;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

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
 *   - TeleOp should call update(...) every loop with live launcher/aim readiness.
 *   - Auto helpers may block on isIdle() while calling update(...) inside loops.
 */
public class FiringController {

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

    private boolean continuousRequested = false;
    private boolean sprayLike = false;
    private boolean firstShotInStream = true;

    private boolean autoAimAllowed = true;
    private long autoAimWindowMs = 0L;
    private long autoAimStartMs = 0L;

    private SharedRobotTuning.HoldFireForRpmMode rpmGateMode = SharedRobotTuning.HoldFireForRpmMode.ALL;

    private boolean intakeSuppressed = false;
    private boolean intakeRestoreState = false;

    private long feedLeadReadyAtMs = 0L;
    private long feedingStartMs = 0L;
    private long feedStopReturnAtMs = 0L;

    private boolean feedMotorStarted = false;
    private long feedMotorStartedAtMs = 0L;

    public FiringController(Feed feed, Intake intake) {
        this.feed = feed;
        this.intake = intake;
    }

    public void reset() {
        state = State.IDLE;
        mode = Mode.SINGLE;
        stateStartMs = 0L;
        continuousRequested = false;
        sprayLike = false;
        firstShotInStream = true;
        autoAimAllowed = true;
        autoAimWindowMs = 0L;
        autoAimStartMs = 0L;
        rpmGateMode = SharedRobotTuning.HoldFireForRpmMode.ALL;
        intakeSuppressed = false;
        intakeRestoreState = false;
        feedLeadReadyAtMs = 0L;
        feedingStartMs = 0L;
        feedStopReturnAtMs = 0L;
        feedMotorStarted = false;
        feedMotorStartedAtMs = 0L;
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
                       boolean launcherReady,
                       double targetRpm,
                       double leftRpm,
                       double rightRpm,
                       boolean intakeDesiredOn) {
        intakeRestoreState = intakeDesiredOn;

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
                } else {
                    transition(State.RPM_WINDOW, nowMs);
                }
                break;
            case AUTOAIM_WINDOW:
                ensureIntakeOn();
                if (aimReady || isAutoAimTimeout(nowMs)) {
                    transition(State.RPM_WINDOW, nowMs);
                }
                break;
            case RPM_WINDOW:
                ensureIntakeOn();
                if (sprayLike || !shouldHoldForRpm()) {
                    transition(State.FEEDSTOP_OPEN, nowMs);
                } else if (launcherReady) {
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
                if (continuousRequested) {
                    transition(State.RECOVERING, nowMs);
                } else {
                    feedStopReturnAtMs = nowMs + Math.max(0L, feed != null ? feed.getReleaseHoldMs() : 0L);
                    transition(State.FEEDSTOP_RETURNS, nowMs);
                }
                break;
            case FEEDSTOP_RETURNS:
                suppressIntake();
                if (nowMs >= feedStopReturnAtMs) {
                    if (feed != null) {
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
                if (launcherReady) {
                    if (continuousRequested) {
                        transition(State.FIRE_REQUESTED, nowMs);
                    } else {
                        restoreIntake();
                        transition(State.IDLE, nowMs);
                    }
                }
                break;
            default:
                break;
        }
    }

    private void transition(State next, long nowMs) {
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

    private boolean shotDetected(double targetRpm, double leftRpm, double rightRpm) {
        if (targetRpm <= 0.0 || !Double.isFinite(targetRpm)) {
            return false;
        }
        double threshold = Math.max(0.0, FeedTuning.FIRING_DROP_RPM_THRESHOLD);
        return (targetRpm - leftRpm) >= threshold
                || (targetRpm - rightRpm) >= threshold;
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
