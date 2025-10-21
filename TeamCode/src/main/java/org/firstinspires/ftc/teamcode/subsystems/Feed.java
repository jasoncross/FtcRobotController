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
 *
 * NOTES:
 * - Motor is set to BRAKE to hold position/pressure when stopped.
 * - If you add encoder-based feed later, add advance/return ticks and RUN_TO_POSITION logic here.
 */
public class Feed {
    public double firePower = 0.9;
    public int fireTimeMs   = 200;
    public int minCycleMs   = 300;

    private final DcMotorEx motor;
    private long lastFire = 0;

    public Feed(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, "FeedMotor");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public boolean canFire() {
        return System.currentTimeMillis() - lastFire >= minCycleMs;
    }

    public void feedOnceBlocking() {
        if (!canFire()) return;
        lastFire = System.currentTimeMillis();
        motor.setPower(firePower);
        sleep(fireTimeMs);
        motor.setPower(0);
    }

    private void sleep(int ms) { try { Thread.sleep(ms); } catch (InterruptedException ignored) {} }
}
