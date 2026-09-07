package org.firstinspires.ftc.teamcode.odometry;

/** Pose in a caller-defined field frame: inches, heading degrees CCW; no field origin assumed. */
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
