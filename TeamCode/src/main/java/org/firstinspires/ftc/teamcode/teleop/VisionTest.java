package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.vision.VisionFactory;
import org.firstinspires.ftc.teamcode.vision.VisionTargetProvider;
import org.firstinspires.ftc.teamcode.vision.TargetObservation;

@TeleOp(name = "BIOBUZZ: Vision Test", group = "BIOBUZZ")
@Disabled // Choose/configure the webcam or Limelight before enabling. No drive hardware is needed.
public class VisionTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        try (VisionTargetProvider vision = VisionFactory.create(hardwareMap)) {
            telemetry.addLine("Vision only. Webcam pose needs tag metadata; Limelight provides bearing.");
            telemetry.update();
            while (opModeInInit()) {
                vision.update();
                telemetry.addData("Vision", vision.getStatus());
                telemetry.update();
                sleep(20);
            }
            waitForStart();
            while (opModeIsActive()) {
                vision.update();
                telemetry.addData("Vision", vision.getStatus());
                for (TargetObservation tag : vision.getTargets()) {
                    telemetry.addData("Tag " + tag.id, "bearing %.1f deg, distance %.2f m", tag.bearingDeg, tag.distanceMeters);
                }
                telemetry.update();
                sleep(20);
            }
        }
    }
}
