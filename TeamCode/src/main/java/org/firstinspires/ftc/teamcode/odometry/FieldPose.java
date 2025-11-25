package org.firstinspires.ftc.teamcode.odometry;

/*
 * FILE: FieldPose.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/odometry/
 *
 * PURPOSE
 *   - Lightweight pose container (X, Y, heading degrees) expressed in the
 *     shared odometry coordinate system. Heading follows the Drivebase/IMU
 *     convention: 0° faces the target wall (+Y), +CCW.
 */
public class FieldPose {
    public double x;
    public double y;
    public double headingDeg;

    public FieldPose() {
        this(0, 0, 0);
    }

    public FieldPose(double x, double y, double headingDeg) {
        this.x = x;
        this.y = y;
        this.headingDeg = headingDeg;
    }

    public FieldPose copy() {
        return new FieldPose(x, y, headingDeg);
    }
}
