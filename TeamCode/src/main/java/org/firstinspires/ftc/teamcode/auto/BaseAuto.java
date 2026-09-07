package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "BIOBUZZ: Auto Template", group = "BIOBUZZ",
        preselectTeleOp = "BIOBUZZ: Base Drive")
@Disabled // Add the new robot's hardware and actions before enabling.
public class BaseAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry.addLine("Autonomous template: no hardware or movement configured.");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;
        // Add autonomous actions here after calibrating the new robot.
        // Check opModeIsActive() in action loops and stop actuators in finally.
    }
}
