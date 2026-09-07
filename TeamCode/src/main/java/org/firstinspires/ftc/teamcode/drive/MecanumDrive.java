package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

/** Robot-centric drive: positive forward, strafe right, and clockwise rotation. */
public final class MecanumDrive {
    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;

    public MecanumDrive(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotor.class, RobotConfig.FRONT_LEFT);
        frontRight = hardwareMap.get(DcMotor.class, RobotConfig.FRONT_RIGHT);
        backLeft = hardwareMap.get(DcMotor.class, RobotConfig.BACK_LEFT);
        backRight = hardwareMap.get(DcMotor.class, RobotConfig.BACK_RIGHT);
        stop();
        frontLeft.setDirection(RobotConfig.FRONT_LEFT_DIRECTION);
        frontRight.setDirection(RobotConfig.FRONT_RIGHT_DIRECTION);
        backLeft.setDirection(RobotConfig.BACK_LEFT_DIRECTION);
        backRight.setDirection(RobotConfig.BACK_RIGHT_DIRECTION);
        for (DcMotor motor : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void drive(double forward, double strafeRight, double clockwise) {
        drive(forward, strafeRight, clockwise, 1.0);
    }

    public void drive(double forward, double strafeRight, double clockwise, double speedScale) {
        double[] powers = MecanumMixer.mix(forward, strafeRight, clockwise,
                RobotConfig.DRIVE_POWER_LIMIT * Math.max(0, Math.min(1, speedScale)));
        frontLeft.setPower(powers[0]);
        frontRight.setPower(powers[1]);
        backLeft.setPower(powers[2]);
        backRight.setPower(powers[3]);
    }

    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
