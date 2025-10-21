package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Alliance;

/*
 * FILE: Auto_Blue_Target.java
 * PURPOSE:
 * - Blue-side target route example (mirror of Red).
 */
@Autonomous(name="Auto: Blue Target", group="Auto", preselectTeleOp="TeleOp - Blue")
public class Auto_Blue_Target extends BaseAuto {
    @Override protected Alliance alliance() { return Alliance.BLUE; }

    @Override
    protected void runSequence() {
        drive.move(24, 0, 0.6);
        drive.turn(-30, 0.5);
        drive.move(12, -90, 0.6);
    }
}
