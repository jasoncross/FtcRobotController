package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/*
 * FILE: TagAimController.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vision/
 *
 * PURPOSE:
 * - Compute a rotation/twist correction that aims the robot at the visible tag,
 *   using the detection's horizontal bearing (degrees).
 *
 * API:
 * - turnPower(det): returns a twist value in [-1, 1] to keep heading on target.
 * - headingDeg(det): convenience for telemetry (NaN when null).
 * - distanceMeters(det): convenience for telemetry (NaN when null).
 *
 * TUNING:
 * - kP: responsiveness (increase if too sluggish).
 * - kD: damping (increase if oscillates).
 * - Clamp/Deadband: limit max twist and stop hunting near zero error.
 */
public class TagAimController {
    private double kP = 0.02;
    private double kD = 0.003;
    private double lastErrorDeg = 0.0;

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
