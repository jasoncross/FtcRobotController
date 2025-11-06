package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.FeedStopConfig;
import org.firstinspires.ftc.teamcode.config.FeedTuning;

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
    public double firePower = FeedTuning.FIRE_POWER; // Shared motor power; referenced by BaseAuto.fireN() + TeleOp bindings
    public int fireTimeMs   = FeedTuning.FIRE_TIME_MS;  // Duration of each feed pulse (ms); ensure sequences allow recovery time
    public int minCycleMs   = FeedTuning.MIN_CYCLE_MS;  // Minimum delay between feeds; prevents double-fire even if buttons spammed
    public double idleHoldPower = FeedTuning.IDLE_HOLD_POWER; // Counter-rotation while idle (0 = BRAKE only)

    private final DcMotorEx motor;
    private long lastFire = 0;
    private boolean idleHoldActive = false;

    private ServoImplEx feedStop;
    private Telemetry feedStopTelemetry;
    private boolean feedStopReady = false;
    private FeedStopState feedStopState = FeedStopState.UNKNOWN;
    private double blockPosition = FeedStopConfig.BLOCK_POS;
    private double releasePosition = FeedStopConfig.RELEASE_POS;
    private long releaseHoldMs = FeedStopConfig.RELEASE_HOLD_MS;
    private long fireLeadMs = FeedStopConfig.FIRE_LEAD_MS;
    private long releaseUntilMs = 0L;
    private long feedAllowedAfterMs = 0L;

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
        setBlock();
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
            feedStop.scaleRange(FeedStopConfig.SCALE_MIN, FeedStopConfig.SCALE_MAX);
            feedStopReady = true;
            setTunables(FeedStopConfig.BLOCK_POS, FeedStopConfig.RELEASE_POS,
                    FeedStopConfig.RELEASE_HOLD_MS, FeedStopConfig.FIRE_LEAD_MS);
            setBlock();
        } catch (Exception ex) {
            feedStopReady = false;
            if (this.feedStopTelemetry != null) {
                this.feedStopTelemetry.addLine("FeedStop init failed: " + ex.getMessage());
            }
        }
    }

    /** Immediately commands the servo to the BLOCK position. */
    public void setBlock() {
        releaseUntilMs = 0L;
        feedAllowedAfterMs = 0L;
        if (!feedStopReady || feedStop == null) {
            feedStopState = FeedStopState.UNKNOWN;
            return;
        }
        feedStop.setPosition(clip(blockPosition));
        feedStopState = FeedStopState.BLOCK;
    }

    /** Immediately commands the servo to the RELEASE position. */
    public void setRelease() {
        if (!feedStopReady || feedStop == null) {
            feedStopState = FeedStopState.UNKNOWN;
            return;
        }
        feedStop.setPosition(clip(releasePosition));
        feedStopState = FeedStopState.RELEASE;
    }

    /** Request a release window, extending the hold timer and enforcing fire lead time. */
    public void requestReleaseHold() {
        if (!feedStopReady || feedStop == null) return;
        long now = System.currentTimeMillis();
        releaseUntilMs = now + Math.max(0L, releaseHoldMs);
        feedAllowedAfterMs = now + Math.max(0L, fireLeadMs);
        setRelease();
    }

    /** Update servo state machine; call every loop. */
    public void update() {
        if (!feedStopReady || feedStop == null) return;
        if (feedStopState == FeedStopState.RELEASE && releaseUntilMs > 0) {
            long now = System.currentTimeMillis();
            if (now >= releaseUntilMs) {
                setBlock();
            }
        }
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

    /** Returns the configured fire lead time (ms). */
    public long getFireLeadMs() {
        return Math.max(0L, fireLeadMs);
    }

    /** Current FeedStop gate state. */
    public FeedStopState getFeedStopState() {
        return feedStopState;
    }

    /** Update tunables at runtime. */
    public void setTunables(double block, double release, long holdMs, long leadMs) {
        blockPosition = clip(block);
        releasePosition = clip(release);
        releaseHoldMs = Math.max(0L, holdMs);
        fireLeadMs = Math.max(0L, leadMs);
        if (feedStopReady && feedStop != null) {
            if (feedStopState == FeedStopState.RELEASE) {
                feedStop.setPosition(clip(releasePosition));
            } else {
                feedStop.setPosition(clip(blockPosition));
            }
        }
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
        if (!canFire()) return;
        lastFire = System.currentTimeMillis();
        applyFeedLeadDelay();
        applySafetyConfig();
        motor.setPower(firePower);
        update();
        sleep(fireTimeMs);
        applyIdleHoldPower();
        update();
    }

    /** Immediately stops the feed motor. */
    public void stop() {
        applySafetyConfig();
        applyIdleHoldPower();
        setBlock();
    }

    /** Force BRAKE zero-power behavior with no idle counter-rotation. */
    public void applyBrakeHold() {
        idleHoldActive = false;
        applySafetyConfig();
        motor.setPower(0.0);
        setBlock();
    }

    /** Helper exposed for safety fallbacks and testing. */
    public void setPower(double p) {
        applySafetyConfig();
        motor.setPower(p);
    }

    private void sleep(int ms) {
        try { Thread.sleep(ms); } catch (InterruptedException ignored) {}
    }

    private void applyFeedLeadDelay() {
        if (!feedStopReady || feedStop == null) return;
        long now = System.currentTimeMillis();
        long waitMs = feedAllowedAfterMs - now;
        if (waitMs <= 0) return;
        while (waitMs > 0) {
            long chunk = Math.min(waitMs, 20);
            sleep((int) chunk);
            update();
            now = System.currentTimeMillis();
            waitMs = feedAllowedAfterMs - now;
        }
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

    private double clip(double v) {
        if (Double.isNaN(v)) return 0.0;
        return Math.max(0.0, Math.min(1.0, v));
    }
}
