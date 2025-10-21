package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
 * FILE: Launcher.java
 * LOCATION: teamcode/.../subsystems/
 *
 * PURPOSE:
 * - Dual-flywheel launcher control. Exposes simple RPM target, stop, and "at speed" check.
 * - Wire to two motors: "FlywheelLeft", "FlywheelRight".
 *
 * TUNABLES:
 * - velocityCoefficient: scale factor mapping requested RPM to motor setVelocity.
 *   If you use setVelocity( ticks/sec ), you'll need TICKS_PER_REV to compute it properly.
 * - bottomRPM / topRPM: clamp range for manual mode mapping.
 * - atSpeedToleranceRPM: how close actual RPM must be to target to allow feeding.
 *
 * IMPORTANT FUNCTIONS:
 * - setTargetRpm(double rpm): sets desired flywheel RPM (clamped).
 * - stop(): stops the flywheels.
 * - isAtSpeed(double tolRpm): returns true if measured speed within tolerance.
 *
 * NOTES:
 * - This stub uses simple open-loop setPower(). If you prefer closed-loop, use setVelocity()
 *   with proper ticks/sec conversion and implement getCurrentRpm().
 */
public class Launcher {
    private final DcMotorEx left, right;

    public double bottomRPM = 2000;
    public double topRPM = 4500;
    public double atSpeedToleranceRPM = 100;
    public double targetRpm = 0;

    // If switching to setVelocity, set this properly and read getVelocity().
    private final boolean useOpenLoopPower = true;
    private double velocityCoefficient = 1.0; // placeholder if you implement setVelocity

    public Launcher(HardwareMap hw) {
        left  = hw.get(DcMotorEx.class, "FlywheelLeft");
        right = hw.get(DcMotorEx.class, "FlywheelRight");

        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.FORWARD);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setTargetRpm(double rpm) {
        targetRpm = clamp(rpm, bottomRPM, topRPM);
        if (useOpenLoopPower) {
            // crude mapping: scale RPM to power in [0..1]
            double p = (targetRpm - bottomRPM) / Math.max(1, (topRPM - bottomRPM));
            p = clamp(p, 0.0, 1.0);
            left.setPower(p);
            right.setPower(p);
        } else {
            // Example if using setVelocity (ticks/sec):
            double ticksPerSec = targetRpm * velocityCoefficient;
            left.setVelocity(ticksPerSec);
            right.setVelocity(ticksPerSec);
        }
    }

    public void stop() {
        left.setPower(0);
        right.setPower(0);
        targetRpm = 0;
    }

    public boolean isAtSpeed(double tolRpm) {
        // If open-loop, approximate using target. Replace with encoder-derived RPM for accuracy.
        double tol = (tolRpm > 0) ? tolRpm : atSpeedToleranceRPM;
        return Math.abs(targetRpm - getCurrentRpm()) <= tol;
    }

    public double getCurrentRpm() {
        // Placeholder. Implement using getVelocity() ticks/sec → RPM conversion for real data.
        return targetRpm;
    }

    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
}
