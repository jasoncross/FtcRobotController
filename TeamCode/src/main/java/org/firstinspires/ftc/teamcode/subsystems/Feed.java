package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
 *       • Shared across modes; increase toward 1.0 when rings stick, lower toward
 *         0.7 if jams occur.
 *   - fireTimeMs
 *       • Duration of the feed pulse in milliseconds (450–650 typical).
 *       • Coordinate with SharedRobotTuning.SHOT_BETWEEN_MS so cadence leaves
 *         enough recovery time.
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
 *     ring stack when idle.
 *   - Future encoder-based feeds can extend this class by replacing the timed
 *     sleep with RUN_TO_POSITION logic.
 */
public class Feed {

    // CHANGES (2025-10-30): Locked zero-power BRAKE + RUN_WITHOUT_ENCODER and guard before each power command.
    public double firePower = FeedTuning.FIRE_POWER; // Shared motor power; referenced by BaseAuto.fireN() + TeleOp bindings
    public int fireTimeMs   = FeedTuning.FIRE_TIME_MS;  // Duration of each feed pulse (ms); coordinate with SHOT_BETWEEN_MS cadence
    public int minCycleMs   = FeedTuning.MIN_CYCLE_MS;  // Minimum delay between feeds; prevents double-fire even if buttons spammed

    private final DcMotorEx motor;
    private long lastFire = 0;

    public Feed(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, "FeedMotor");
        applySafetyConfig();
    }

    /** Returns true if enough time has passed since last feed to fire again. */
    public boolean canFire() {
        return System.currentTimeMillis() - lastFire >= minCycleMs;
    }

    /** Timed feed (blocking for ~fireTimeMs). */
    public void feedOnceBlocking() {
        if (!canFire()) return;
        lastFire = System.currentTimeMillis();
        applySafetyConfig();
        motor.setPower(firePower);
        sleep(fireTimeMs);
        motor.setPower(0);
    }

    /** Immediately stops the feed motor. */
    public void stop() {
        applySafetyConfig();
        motor.setPower(0);
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
}
