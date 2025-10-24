package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
 * FILE: Feed.java
 * LOCATION: teamcode/.../subsystems/
 *
 * PURPOSE:
 * - Controls the "FeedMotor" that holds a ball and advances briefly to feed one ball into flywheels.
 * - Default implementation is timed (simple and reliable).
 *
 * TUNABLES:
 * - firePower: motor power during feed (0.6–1.0 typical).
 * - fireTimeMs: duration of feed motion (150–300 ms typical).
 * - minCycleMs: debounce between consecutive feeds to prevent double-feeds.
 *
 * IMPORTANT FUNCTIONS:
 * - feedOnceBlocking(): perform one feed cycle (advance briefly then stop).
 * - canFire(): true if debounce interval has passed.
 * - stop(): immediately stop the feed motor (used by TeleOp StopAll).
 *
 * NOTES:
 * - Motor is set to BRAKE to hold position/pressure when stopped.
 * - If you add encoder-based feed later, add advance/return ticks and RUN_TO_POSITION logic here.
 * - NEW (2025-10-23): Added stop() and setPower(double) for integration with StopAll.
 */
public class Feed {
    public double firePower = 0.9;
    public int fireTimeMs   = 600;
    public int minCycleMs   = 300;

    private final DcMotorEx motor;
    private long lastFire = 0;

    public Feed(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, "FeedMotor");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /** Returns true if enough time has passed since last feed to fire again. */
    public boolean canFire() {
        return System.currentTimeMillis() - lastFire >= minCycleMs;
    }

    /** Timed feed (blocking for ~fireTimeMs). */
    public void feedOnceBlocking() {
        if (!canFire()) return;
        lastFire = System.currentTimeMillis();
        motor.setPower(firePower);
        sleep(fireTimeMs);
        motor.setPower(0);
    }

    /** Immediately stops the feed motor. */
    public void stop() {
        motor.setPower(0);
    }

    /** Helper exposed for safety fallbacks and testing. */
    public void setPower(double p) {
        motor.setPower(p);
    }

    private void sleep(int ms) {
        try { Thread.sleep(ms); } catch (InterruptedException ignored) {}
    }
}
