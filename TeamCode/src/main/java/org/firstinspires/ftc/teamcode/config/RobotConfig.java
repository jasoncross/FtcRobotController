package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

/** Starting values to review against the new robot before enabling drive OpModes. */
public final class RobotConfig {
    private RobotConfig() { }

    public static final String FRONT_LEFT = "front_left";
    public static final String FRONT_RIGHT = "front_right";
    public static final String BACK_LEFT = "back_left";
    public static final String BACK_RIGHT = "back_right";
    public static final String IMU_NAME = "imu";

    // Conventional starting directions, not calibration from the DECODE robot.
    public static final DcMotorSimple.Direction FRONT_LEFT_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction FRONT_RIGHT_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction BACK_LEFT_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction BACK_RIGHT_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final double DRIVE_POWER_LIMIT = 0.4;

    // Vision is optional during driving; the vision test uses only this camera.
    public static final boolean VISION_ENABLED = false;
    public static final String WEBCAM_NAME = "Webcam 1";
}
