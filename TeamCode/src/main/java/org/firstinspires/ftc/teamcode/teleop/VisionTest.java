package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.vision.AprilTagVision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "BIOBUZZ: Vision Test", group = "BIOBUZZ")
@Disabled // Configure the webcam before enabling. No drive hardware is needed.
public class VisionTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        try (AprilTagVision vision = new AprilTagVision(hardwareMap)) {
            telemetry.addLine("Vision only. Default empty library reports tag IDs and image centers.");
            telemetry.update();
            waitForStart();
            while (opModeIsActive()) {
                telemetry.addData("Camera", vision.getCameraState());
                for (AprilTagDetection tag : vision.getDetections()) {
                    telemetry.addData("Tag " + tag.id, "center %.0f, %.0f px", tag.center.x, tag.center.y);
                }
                telemetry.update();
                sleep(20);
            }
        }
    }
}
