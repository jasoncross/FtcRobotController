package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.IntakeTuning;

/*
 * FILE: Intake.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/
 *
 * PURPOSE
 *   - Drive the intake motor with an encoder-aware state machine that allows the
 *     designed "packing jam" against the firing column while protecting the motor
 *     from true stalls.
 *   - Classify the column into FREE_FLOW → PACKING → SATURATED → JAMMED phases so
 *     TeleOp and Auto keep three balls compressed without holding a hard stall.
 *   - Coordinate with Feed activity so the intake backs off during launcher shots.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Intake power & driver defaults)
 *   - fillPower / packingPower / holdPower / feedActiveHoldPower
 *       • Shared power levels for the FREE_FLOW, PACKING, SATURATED, and
 *         "Feed is active" hold behaviors.
 *   - holdPulsePeriodMs / holdPulseOnMs
 *       • Duty cycle of the SATURATED hold pulse that maintains column pressure
 *         without continuous stall current.
 *   - sampleIntervalMs / freeDeltaTicks / contactDeltaTicks / stallDeltaTicks
 *       • Encoder sampling cadence + motion thresholds that separate normal flow
 *         from intentional packing and true stalls.
 *   - packingRangeTicks
 *       • Encoder travel allowed after the first contact to compress up to three
 *         balls before declaring the column SATURATED.
 *   - stallDebounceSamples / jamRecoveryPauseMs
 *       • Consecutive stall samples required before declaring a JAM and the
 *         cooldown before retrying forward flow.
 *
 * METHODS
 *   - toggle()/set(boolean)
 *       • Flip or directly command the intake state, resetting the classifier when
 *         re-enabled.
 *   - update(boolean feedActive)
 *       • Sample encoder delta, advance the state machine, and apply the correct
 *         motor power while honoring feed/launcher coordination.
 *   - getState()
 *       • Exposed for telemetry so drivers can see FREE/PACK/SAT/JAM transitions.
 *   - stop()/applyBrakeHold()
 *       • Force the intake OFF and reset encoder monitoring (StopAll safety hooks).
 *
 * NOTES
 *   - TeleOpAllianceBase.DEFAULT_INTAKE_ENABLED still governs whether TeleOp
 *     starts with the intake on; safeInit() keeps the motor motionless until START.
 *   - All tunables live in config/IntakeTuning.java so adjustments stay centralized.
 *
 * CHANGES (2025-10-31): Added safeInit to guarantee zero power during INIT.
 * CHANGES (2025-11-04): Added applyBrakeHold() to reinforce BRAKE during StopAll events.
 * CHANGES (2025-11-16): Replaced the simple on/off control with an encoder-aware
 *                       jam classifier (FREE_FLOW, PACKING, SATURATED, JAMMED) plus
 *                       hold-power pulsing and Feed-aware load shedding.
 * CHANGES (2025-11-17): Allow PACKING state to trigger the jam detection debounce so
 *                       hard stalls that occur before saturation now flip directly
 *                       into JAMMED and recover automatically.
 * CHANGES (2025-11-18): Track packing progress so long pauses with no encoder
 *                       movement now register as JAMMED even when the motor jitters
 *                       just above the stall threshold.
 * CHANGES (2025-11-25): Added reverse pulse support for a triple-tap RB gesture in
 *                       TeleOp to clear jams without reconfiguring tunables.
 * CHANGES (2025-11-25): Converted the reverse pulse into a latched reverse mode
 *                       that runs until the intake toggle is tapped again, then
 *                       restores the previous on/off state and aligned changelog
 *                       dates with the 2025-11-25 release.
 */
public class Intake {
    private final DcMotorEx motor;

    private boolean on = false;
    private FlowState state = FlowState.OFF;
    private int lastSampleTicks = 0;
    private long lastSampleTimeMs = 0L;
    private int packStartTicks = 0;
    private int stallSampleCount = 0;
    private int contactSampleCount = 0;
    private int packingNoProgressSamples = 0;
    private int lastPackingTravelTicks = 0;
    private long jamRecoverUntilMs = 0L;
    private long holdPulseAnchorMs = 0L;
    private double lastCommandedPower = 0.0;
    private boolean reversing = false;
    private boolean reverseResumeOn = false;

    private static final double EPS = 1e-6;

    /** Intake flow classification for telemetry. */
    public enum FlowState {
        OFF,
        FREE_FLOW,
        PACKING,
        SATURATED,
        JAMMED,
        REVERSE
    }

    public Intake(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, "Intake");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        try {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception ignored) {
            // Practice bots without encoders can still report positions even if RUN_WITHOUT remains set.
        }
        safeInit();
    }

    /** Ensure intake stays idle during INIT. */
    public void safeInit() {
        motor.setPower(0.0);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        on = false;
        state = FlowState.OFF;
        jamRecoverUntilMs = 0L;
        lastSampleTimeMs = 0L;
        lastSampleTicks = motor.getCurrentPosition();
        packStartTicks = lastSampleTicks;
        stallSampleCount = 0;
        contactSampleCount = 0;
        packingNoProgressSamples = 0;
        lastPackingTravelTicks = 0;
        holdPulseAnchorMs = System.currentTimeMillis();
        lastCommandedPower = 0.0;
        reversing = false;
        reverseResumeOn = false;
    }

    /** Toggle intake state (useful for button bindings). */
    public void toggle() { set(!on); }

    /** Set intake to run or stop at the configured power (reset classifier on enable). */
    public void set(boolean enable) {
        on = enable;
        reversing = false;
        if (enable) {
            state = FlowState.FREE_FLOW;
            jamRecoverUntilMs = 0L;
            stallSampleCount = 0;
            contactSampleCount = 0;
            packingNoProgressSamples = 0;
            lastSampleTicks = motor.getCurrentPosition();
            packStartTicks = lastSampleTicks;
            lastPackingTravelTicks = 0;
            lastSampleTimeMs = System.currentTimeMillis();
            holdPulseAnchorMs = lastSampleTimeMs;
            update(false);
        } else {
            state = FlowState.OFF;
            jamRecoverUntilMs = 0L;
            stallSampleCount = 0;
            contactSampleCount = 0;
            packingNoProgressSamples = 0;
            lastPackingTravelTicks = 0;
            applyPower(0.0);
        }
    }

    /** Begin a latched reverse run that continues until resumeFromReverse(...) is called. */
    public void startReverse(boolean resumeOn) {
        reversing = true;
        reverseResumeOn = resumeOn;
        on = true;
        state = FlowState.REVERSE;
        jamRecoverUntilMs = 0L;
        stallSampleCount = 0;
        contactSampleCount = 0;
        packingNoProgressSamples = 0;
        lastSampleTicks = motor.getCurrentPosition();
        packStartTicks = lastSampleTicks;
        lastPackingTravelTicks = 0;
        lastSampleTimeMs = System.currentTimeMillis();
        holdPulseAnchorMs = lastSampleTimeMs;
        applyPower(-Math.abs(IntakeTuning.REVERSE_POWER));
    }

    /** Exit reverse mode and restore the intake to its saved on/off state. */
    public void resumeFromReverse() {
        if (!reversing) {
            return;
        }
        reversing = false;
        on = reverseResumeOn;
        if (!on) {
            state = FlowState.OFF;
            applyPower(0.0);
            return;
        }
        state = FlowState.FREE_FLOW;
        jamRecoverUntilMs = 0L;
        stallSampleCount = 0;
        contactSampleCount = 0;
        packingNoProgressSamples = 0;
        lastSampleTicks = motor.getCurrentPosition();
        packStartTicks = lastSampleTicks;
        lastPackingTravelTicks = 0;
        lastSampleTimeMs = System.currentTimeMillis();
        holdPulseAnchorMs = lastSampleTimeMs;
        update(false);
    }

    /** Advance the encoder-based classifier and apply motor power. */
    public void update() { update(false); }

    /**
     * @param feedCycleActive true when the Feed subsystem is in the middle of a cycle.
     *                        Intake drops to FEED_ACTIVE hold power whenever this is true.
     */
    public void update(boolean feedCycleActive) {
        long now = System.currentTimeMillis();

        if (reversing) {
            state = FlowState.REVERSE;
            applyPower(-Math.abs(IntakeTuning.REVERSE_POWER));
            return;
        }

        if (!on) {
            state = FlowState.OFF;
            applyPower(0.0);
            return;
        }

        if (state == FlowState.JAMMED && now >= jamRecoverUntilMs) {
            state = FlowState.FREE_FLOW;
            stallSampleCount = 0;
            contactSampleCount = 0;
            packingNoProgressSamples = 0;
            lastSampleTicks = motor.getCurrentPosition();
            packStartTicks = lastSampleTicks;
            lastPackingTravelTicks = 0;
            holdPulseAnchorMs = now;
        }

        if (!feedCycleActive) {
            maybeSampleEncoder(now);
        }

        applyStatePower(feedCycleActive, now);
    }

    /** @return true when intake motor is currently running. */
    public boolean isOn() { return on; }

    /** @return true when the intake is latched in reverse mode. */
    public boolean isReversing() { return reversing; }

    /** @return current state for telemetry. */
    public FlowState getState() { return state; }

    /** @return formatted telemetry summary (e.g., "ON – PACKING"). */
    public String getTelemetrySummary() {
        if (!on) {
            return "OFF";
        }
        String label = state.name().replace('_', ' ');
        return "ON – " + label;
    }

    /** Immediately turns intake OFF (safe to call repeatedly). */
    public void stop() {
        set(false);
    }

    /** Ensure BRAKE zero-power behavior is set and hold motor idle. */
    public void applyBrakeHold() {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        set(false);
    }

    private void maybeSampleEncoder(long now) {
        int interval = Math.max(1, IntakeTuning.SAMPLE_INTERVAL_MS);
        if ((now - lastSampleTimeMs) < interval) {
            return;
        }
        lastSampleTimeMs = now;
        int currentTicks = motor.getCurrentPosition();
        int delta = Math.abs(currentTicks - lastSampleTicks);
        lastSampleTicks = currentTicks;

        updateContactCounters(delta);
        classifyFlow(delta, currentTicks, now);
    }

    private void updateContactCounters(int delta) {
        if (delta <= Math.max(1, IntakeTuning.CONTACT_DELTA_TICKS)) {
            contactSampleCount++;
        } else {
            contactSampleCount = 0;
        }

        if (delta <= Math.max(1, IntakeTuning.STALL_DELTA_TICKS)) {
            stallSampleCount++;
        } else {
            stallSampleCount = 0;
        }
    }

    private void classifyFlow(int delta, int currentTicks, long now) {
        int freeThreshold = Math.max(IntakeTuning.CONTACT_DELTA_TICKS + 1, IntakeTuning.FREE_DELTA_TICKS);
        int packingTravel = Math.max(1, IntakeTuning.PACKING_RANGE_TICKS);
        int stallSamplesNeeded = Math.max(1, IntakeTuning.STALL_DEBOUNCE_SAMPLES);

        switch (state) {
            case FREE_FLOW:
                if (delta <= Math.max(1, IntakeTuning.CONTACT_DELTA_TICKS) && contactSampleCount >= 2) {
                    enterPacking(currentTicks, now);
                }
                break;
            case PACKING:
                int travel = Math.abs(currentTicks - packStartTicks);
                if (travel > lastPackingTravelTicks) {
                    lastPackingTravelTicks = travel;
                    packingNoProgressSamples = 0;
                } else {
                    packingNoProgressSamples++;
                }
                if (travel >= packingTravel) {
                    enterSaturated(now);
                } else if (stallSampleCount >= stallSamplesNeeded || packingNoProgressSamples >= stallSamplesNeeded) {
                    enterJammed(now);
                } else if (delta >= freeThreshold) {
                    // Column moved freely again—reset to FREE_FLOW so packing restarts from new reference.
                    state = FlowState.FREE_FLOW;
                    packStartTicks = currentTicks;
                    contactSampleCount = 0;
                    stallSampleCount = 0;
                    packingNoProgressSamples = 0;
                    lastPackingTravelTicks = 0;
                }
                break;
            case SATURATED:
                if (delta >= freeThreshold) {
                    state = FlowState.FREE_FLOW;
                    packStartTicks = currentTicks;
                    contactSampleCount = 0;
                    stallSampleCount = 0;
                    packingNoProgressSamples = 0;
                    lastPackingTravelTicks = 0;
                    holdPulseAnchorMs = now;
                } else if (stallSampleCount >= stallSamplesNeeded) {
                    enterJammed(now);
                }
                break;
            case JAMMED:
                // Cooldown handled in update().
                break;
            case OFF:
            default:
                break;
        }
    }

    private void enterPacking(int currentTicks, long now) {
        state = FlowState.PACKING;
        packStartTicks = currentTicks;
        stallSampleCount = 0;
        contactSampleCount = 0;
        packingNoProgressSamples = 0;
        lastPackingTravelTicks = 0;
        holdPulseAnchorMs = now;
    }

    private void enterSaturated(long now) {
        state = FlowState.SATURATED;
        stallSampleCount = 0;
        contactSampleCount = 0;
        packingNoProgressSamples = 0;
        lastPackingTravelTicks = 0;
        holdPulseAnchorMs = now;
    }

    private void enterJammed(long now) {
        state = FlowState.JAMMED;
        jamRecoverUntilMs = now + Math.max(0, IntakeTuning.JAM_RECOVERY_PAUSE_MS);
        stallSampleCount = 0;
        contactSampleCount = 0;
        packingNoProgressSamples = 0;
        lastPackingTravelTicks = 0;
        holdPulseAnchorMs = now;
    }

    private void applyStatePower(boolean feedActive, long now) {
        double power;
        if (!on) {
            power = 0.0;
        } else {
            switch (state) {
                case FREE_FLOW:
                    power = IntakeTuning.FILL_POWER;
                    break;
                case PACKING:
                    power = IntakeTuning.PACKING_POWER;
                    break;
                case SATURATED:
                    power = computeHoldPulse(now);
                    break;
                case JAMMED:
                    power = (now >= jamRecoverUntilMs) ? IntakeTuning.FILL_POWER : 0.0;
                    break;
                case REVERSE:
                    power = -Math.abs(IntakeTuning.REVERSE_POWER);
                    break;
                case OFF:
                default:
                    power = 0.0;
                    break;
            }
        }

        if (feedActive) {
            power = Math.min(power, IntakeTuning.FEED_ACTIVE_HOLD_POWER);
        }

        applyPower(power);
    }

    private double computeHoldPulse(long now) {
        int period = IntakeTuning.HOLD_PULSE_PERIOD_MS;
        int onMs = IntakeTuning.HOLD_PULSE_ON_MS;
        if (period <= 0 || onMs <= 0 || onMs >= period) {
            return IntakeTuning.HOLD_POWER;
        }
        long elapsed = Math.max(0, now - holdPulseAnchorMs);
        long mod = elapsed % period;
        return (mod < onMs) ? IntakeTuning.HOLD_POWER : 0.0;
    }

    private void applyPower(double power) {
        if (Math.abs(power) < EPS) {
            power = 0.0;
        }
        if (Math.abs(power - lastCommandedPower) < EPS) {
            return;
        }
        motor.setPower(power);
        lastCommandedPower = power;
    }
}
