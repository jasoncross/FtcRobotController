package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.config.LauncherTuning;

/*
 * FILE: Launcher.java
 * LOCATION: teamcode/.../subsystems/
 *
 * PURPOSE
 *   - Control the dual goBILDA 5202 flywheel motors using the REV Hub’s
 *     closed-loop velocity PIDF so launch speed stays consistent across battery
 *     levels.
 *   - Convert requested RPM into ticks/second and command the hub directly so
 *     both TeleOp and Auto share the same behavior.
 *
 * HARDWARE
 *   - Two motors: "FlywheelLeft" and "FlywheelRight" (goBILDA 5202 6000 RPM).
 *   - Encoders: built-in 28 ticks/rev on each motor (adjust FLYWHEEL_TPR if gear
 *     reduction is added).
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Launcher speed & flywheel control)
 *   - FLYWHEEL_TPR
 *       • Encoder ticks per revolution at the measured shaft.
 *       • Multiply 28 × gear ratio if external reduction is added so RPM math
 *         remains accurate.
 *   - RPM_MIN / RPM_MAX
 *       • Software clamps applied to every RPM request.
 *       • Keep RPM_MAX ≥ the highest AutoRpmConfig calibration RPM and
 *         TeleOpAllianceBase.rpmTop so downstream commands do not saturate.
 *   - PIDF (RUN_USING_ENCODER)
 *       • Closed-loop gains applied at the hub. Start near TunableDirectory’s
 *         P=10, I=3, D=0, F=12 and adjust for wheel mass or overshoot.
 *   - atSpeedToleranceRPM
 *       • Local fallback readiness window when callers omit a tolerance. Align
 *         with SharedRobotTuning.RPM_TOLERANCE so Auto/TeleOp gating matches.
 *
 * METHODS
 *   - setTargetRpm(double rpm)
 *       • Commands a closed-loop velocity target (RPM converted to ticks/sec).
 *   - stop()
 *       • Immediately stops both flywheels (open-loop).
 *   - getCurrentRpm()
 *       • Returns the measured average RPM from both encoders.
 *   - isAtSpeed(double tolRpm)
 *       • Reports readiness using either the provided tolerance or
 *         atSpeedToleranceRPM.
 *
 * NOTES
 *   - TeleOpAllianceBase and AutoAimSpeed continuously call setTargetRpm();
 *     ensure any tuning is validated in both modes.
 *   - The REV hub handles velocity feedback internally, so this class does not
 *     implement its own PID loop.
 * CHANGES (2025-11-19): Exposed left/right RPM readings for telemetry alongside
 *                       the averaged speed so drivers can see wheel balance.
 */

public class Launcher {
    // === CONFIGURATION CONSTANTS ===
    private static final double FLYWHEEL_TPR = LauncherTuning.FLYWHEEL_TPR; // Encoder ticks per revolution; adjust via config
    private static final double RPM_MIN      = LauncherTuning.RPM_MIN;      // Minimum allowed RPM requested by any caller
    // CHANGES (2025-11-15): Updated RPM_MAX guidance to follow the new AutoRpmConfig calibration table.
    private static final double RPM_MAX      = LauncherTuning.RPM_MAX;      // Maximum allowed RPM; keep ≥ highest AutoSpeed calibration RPM & TeleOp rpmTop

    // Default closed-loop PIDF values for REV velocity control.
    // Adjust only if behavior indicates overshoot, oscillation, or slow recovery.
    private static final PIDFCoefficients PIDF = new PIDFCoefficients(
            LauncherTuning.PID_P,  // P gain
            LauncherTuning.PID_I,  // I gain
            LauncherTuning.PID_D,  // D gain
            LauncherTuning.PID_F   // F feedforward
    );

    // === HARDWARE OBJECTS ===
    private final DcMotorEx left;
    private final DcMotorEx right;

    // === STATE ===
    public double targetRpm = 0;                // Last commanded RPM request (TeleOp + Auto)
    public double atSpeedToleranceRPM = LauncherTuning.AT_SPEED_TOLERANCE_RPM;    // Local readiness window; align with shared tuning

    // === CONSTRUCTOR ===
    // CHANGES (2025-10-31): Added safeInit to hold zero RPM during INIT.
    // CHANGES (2025-11-04): Added applyBrakeHold() for StopAll + auto-restoring FLOAT on next command.

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

        safeInit();
    }

    /** Ensure flywheels remain stopped during INIT. */
    public void safeInit() {
        targetRpm = 0;
        left.setPower(0);
        right.setPower(0);
    }

    // === PUBLIC API ===

    /**
     * Command a desired flywheel speed in RPM.
     * Converts RPM → ticks/second and uses closed-loop velocity control.
     */
    public void setTargetRpm(double rpm) {
        ensureCoastMode();
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

    /** Engage BRAKE zero-power behavior and hold both flywheels at zero power. */
    public void applyBrakeHold() {
        targetRpm = 0;
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

    /** Returns the measured RPM for the left flywheel only. */
    public double getLeftRpm() {
        return ticksPerSecToRpm(left.getVelocity());
    }

    /** Returns the measured RPM for the right flywheel only. */
    public double getRightRpm() {
        return ticksPerSecToRpm(right.getVelocity());
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

    /** Restore FLOAT zero-power behavior before commanding velocity. */
    private void ensureCoastMode() {
        if (left.getZeroPowerBehavior() != DcMotor.ZeroPowerBehavior.FLOAT) {
            left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        if (right.getZeroPowerBehavior() != DcMotor.ZeroPowerBehavior.FLOAT) {
            right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }
}
