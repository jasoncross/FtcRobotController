package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Alliance;

/*
 * FILE: Auto_Red_Target.java
 * PURPOSE:
 * - Red-side target route example. Edit runSequence() with your real path.
 * TUNABLES:
 * - Distances, headings, and speeds inside runSequence().
 * IMPORTANT:
 * - preselectTeleOp queues "TeleOp - Red" when Auto ends.
 */
@Autonomous(name="Auto: Red Target", group="Auto", preselectTeleOp="TeleOp - Red")
public class Auto_Red_Target extends BaseAuto {
    @Override protected Alliance alliance() { return Alliance.RED; }

    @Override
    protected void runSequence() {
        drive.move(24, 0, 0.6);
        drive.turn(30, 0.5);
        drive.move(12, 90, 0.6);
        // TODO: spin up launcher, wait isAtSpeed(), feedOnce(), etc.
    }
}
