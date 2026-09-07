package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.AprilTagVision;

@TeleOp(name = "BIOBUZZ: Base Drive", group = "BIOBUZZ")
@Disabled // Review RobotConfig and test wheel directions before enabling.
public class BaseDriveTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        AprilTagVision vision = null;
        try {
            if (RobotConfig.VISION_ENABLED) vision = new AprilTagVision(hardwareMap);
            telemetry.addLine("Left stick: move; right stick X: turn; hold A: stop.");
            telemetry.addData("Drive power limit", RobotConfig.DRIVE_POWER_LIMIT);
            telemetry.update();
            waitForStart();
            while (opModeIsActive()) {
                if (gamepad1.a) {
                    drive.stop();
                } else {
                    drive.drive(deadband(-gamepad1.left_stick_y),
                            deadband(gamepad1.left_stick_x), deadband(gamepad1.right_stick_x));
                }
                if (vision != null) {
                    telemetry.addData("Camera", vision.getCameraState());
                    telemetry.addData("Tags detected", vision.getDetections().size());
                }
                telemetry.addData("Drive", gamepad1.a ? "Stopped" : "Robot-centric");
                telemetry.update();
                idle();
            }
        } finally {
            drive.stop();
            if (vision != null) vision.close();
        }
    }

    private static double deadband(double value) {
        return Math.abs(value) < RobotConfig.STICK_DEADBAND ? 0 : value;
    }
}
