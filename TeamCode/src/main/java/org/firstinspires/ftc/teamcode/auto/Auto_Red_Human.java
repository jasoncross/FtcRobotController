package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Alliance;

/*
 * FILE: Auto_Red_Human.java
 * PURPOSE:
 * - Red-side human-player route example.
 */
@Autonomous(name="Auto: Red Human", group="Auto", preselectTeleOp="TeleOp - Red")
public class Auto_Red_Human extends BaseAuto {
    @Override protected Alliance alliance() { return Alliance.RED; }

    @Override
    protected void runSequence() {
        drive.move(18, 0, 0.6);
        drive.move(18, 90, 0.6);
        drive.turn(-45, 0.5);
    }
}
