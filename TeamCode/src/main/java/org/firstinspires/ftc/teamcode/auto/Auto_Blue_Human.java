package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Alliance;

/*
 * FILE: Auto_Blue_Human.java
 * PURPOSE:
 * - Blue-side human-player route example.
 */
@Autonomous(name="Auto: Blue Human", group="Auto", preselectTeleOp="TeleOp - Blue")
public class Auto_Blue_Human extends BaseAuto {
    @Override protected Alliance alliance() { return Alliance.BLUE; }

    @Override
    protected void runSequence() {
        drive.move(18, 0, 0.6);
        drive.move(18, -90, 0.6);
        drive.turn(45, 0.5);
    }
}
