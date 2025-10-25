package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/*
 * FILE: TagAimController.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vision/
 *
 * PURPOSE
 *   - Translate AprilTag bearing error into a twist command shared by TeleOp
 *     aim assist and Autonomous helpers.
 *   - Provide convenience helpers for telemetry (heading/range).
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → AutoAim)
 *   - kP / kD
 *       • Proportional and derivative gains. Increase kP toward 0.025 for faster
 *         alignment; add kD (~0.004) if oscillations appear.
 *       • Coordinate with SharedRobotTuning.TURN_TWIST_CAP so PD output has room
 *         to act.
 *   - Clamp (±0.6) and deadband (±1.5°)
 *       • Limit twist strength and stop hunting. Expand clamp toward ±0.8 only if
 *         SharedRobotTuning.TURN_TWIST_CAP also increases.
 *
 * METHODS
 *   - setGains(kP, kD)
 *       • Update PD terms to match new tuning.
 *   - turnPower(det)
 *       • Return twist suggestion in [-0.6, 0.6] based on tag error.
 *   - headingDeg(det) / distanceMeters(det)
 *       • Telemetry helpers returning NaN when no tag is present.
 */
public class TagAimController {
    private double kP = 0.02;        // Proportional gain (twist per degree); align with TunableDirectory AutoAim table
    private double kD = 0.003;       // Derivative gain to damp overshoot; increase slightly when oscillations appear
    private double lastErrorDeg = 0.0; // Stored from previous frame for D term

    public void setGains(double kP, double kD) { this.kP = kP; this.kD = kD; }

    /** Returns twist power [-1..1] to align robot to the tag; 0 when det == null or within deadband. */
    public double turnPower(AprilTagDetection det) {
        if (det == null) return 0.0;
        double errDeg = det.ftcPose.bearing;   // + right, - left (SDK convention)
        double deriv  = errDeg - lastErrorDeg; // simple D term (frame-to-frame)
        lastErrorDeg  = errDeg;

        double power = (kP * errDeg) + (kD * deriv);

        // Clamp twist (sane max while still letting you translate at full speed)
        if (power > 0.6) power = 0.6;
        if (power < -0.6) power = -0.6;

        // Deadband to stop hunting
        if (Math.abs(errDeg) < 1.5) power = 0.0;

        return power;
    }

    public static double headingDeg(AprilTagDetection det) {
        return det == null ? Double.NaN : det.ftcPose.bearing;
    }

    public static double distanceMeters(AprilTagDetection det) {
        return det == null ? Double.NaN : det.ftcPose.range;
    }
}
