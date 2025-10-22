package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/*
 * FILE: Launcher.java
 * LOCATION: teamcode/.../subsystems/
 *
 * PURPOSE:
 * - Controls the two 6000 RPM goBILDA 5202 flywheel motors using CLOSED-LOOP velocity control.
 * - Converts target RPM to encoder ticks/second and commands that velocity directly to the hub.
 * - Uses built-in REV Hub PIDF loop to maintain consistent launch speed regardless of battery or load.
 *
 * HARDWARE:
 * - Two motors: "FlywheelLeft" and "FlywheelRight"
 * - Motor type: goBILDA 5202 6000 RPM (bare motor, 28 ticks/rev encoder)
 *
 * FUNCTIONAL SUMMARY:
 * - setTargetRpm(double rpm):   Commands a closed-loop velocity target in RPM.
 * - stop():                     Stops both flywheels immediately.
 * - getCurrentRpm():            Returns measured average RPM from encoders.
 * - isAtSpeed(double tolRpm):   Returns true when within specified RPM tolerance.
 *
 * TUNABLE PARAMETERS:
 * - FLYWHEEL_TPR:  Encoder ticks per revolution at the measured shaft.
 *                  For goBILDA 5202 @ 6000 RPM, use 28.0 (no gearbox).
 *                  If you later add gearing, multiply 28 × gear ratio.
 *
 * - RPM_MIN / RPM_MAX:  Software clamp limits for TeleOp control range.
 *
 * - PIDF:  Closed-loop PIDF coefficients used in RUN_USING_ENCODER mode.
 *   Typical starting point:
 *      P = 10.0   → raise if slow to reach speed; lower if oscillates
 *      I = 3.0    → adds steady-state correction; start small
 *      D = 0.0    → adds damping; increase slightly if overshooting
 *      F = 12.0   → baseline feedforward term (scales open-loop power)
 *
 * NOTES:
 * - Each hub port’s built-in controller handles velocity feedback internally.
 * - You can read current speed with getVelocity() and convert to RPM for telemetry.
 * - Typical target range: 0 – 6000 RPM.
 * - TeleOp should call setTargetRpm() continuously with desired RPM (usually from trigger).
 */

public class Launcher {
    // === CONFIGURATION CONSTANTS ===
    private static final double FLYWHEEL_TPR = 28.0;     // Encoder ticks per revolution
    private static final double RPM_MIN      = 0.0;      // Minimum allowed RPM
    private static final double RPM_MAX      = 6000.0;   // Maximum allowed RPM

    // Default closed-loop PIDF values for REV velocity control.
    // Adjust only if behavior indicates overshoot, oscillation, or slow recovery.
    private static final PIDFCoefficients PIDF = new PIDFCoefficients(
            10.0,  // P gain
            3.0,   // I gain
            0.0,   // D gain
            12.0   // F feedforward
    );

    // === HARDWARE OBJECTS ===
    private final DcMotorEx left;
    private final DcMotorEx right;

    // === STATE ===
    public double targetRpm = 0;                // Last commanded RPM
    public double atSpeedToleranceRPM = 100;    // Window for "ready to fire"

    // === CONSTRUCTOR ===
    public Launcher(HardwareMap hw) {
        // Retrieve motors from configuration
        left  = hw.get(DcMotorEx.class, "FlywheelLeft");
        right = hw.get(DcMotorEx.class, "FlywheelRight");

        // Make both flywheels spin the same physical direction
        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        // Flywheels should coast when power = 0
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Enable encoder-based velocity control and apply PIDF coefficients
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF);
        right.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF);
    }

    // === PUBLIC API ===

    /**
     * Command a desired flywheel speed in RPM.
     * Converts RPM → ticks/second and uses closed-loop velocity control.
     */
    public void setTargetRpm(double rpm) {
        targetRpm = clamp(rpm, RPM_MIN, RPM_MAX);
        double ticksPerSec = rpmToTicksPerSec(targetRpm);

        // One-argument setVelocity() uses ticks/second for DcMotorEx
        left.setVelocity(ticksPerSec);
        right.setVelocity(ticksPerSec);
    }

    /** Immediately stops both flywheels (open loop 0 power). */
    public void stop() {
        targetRpm = 0;
        left.setPower(0);
        right.setPower(0);
    }

    /**
     * Returns true when the measured RPM is within tolerance of target.
     * Useful for "ready-to-fire" gating.
     */
    public boolean isAtSpeed(double tolRpm) {
        double tol = (tolRpm > 0) ? tolRpm : atSpeedToleranceRPM;
        return Math.abs(getCurrentRpm() - targetRpm) <= tol;
    }

    /** Returns the current measured average RPM from both flywheel encoders. */
    public double getCurrentRpm() {
        double leftTPS  = left.getVelocity();   // ticks per second
        double rightTPS = right.getVelocity();  // ticks per second
        double avgTPS   = (leftTPS + rightTPS) / 2.0;
        return ticksPerSecToRpm(avgTPS);
    }

    // === HELPER METHODS ===

    /** Convert RPM → ticks/sec using configured TPR. */
    private static double rpmToTicksPerSec(double rpm) {
        return rpm * FLYWHEEL_TPR / 60.0;
    }

    /** Convert ticks/sec → RPM using configured TPR. */
    private static double ticksPerSecToRpm(double tps) {
        return tps * 60.0 / FLYWHEEL_TPR;
    }

    /** Clamp helper for numeric safety. */
    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
