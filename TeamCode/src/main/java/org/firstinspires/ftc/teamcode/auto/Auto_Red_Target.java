package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Alliance;

/** Red Target — south edge of RED depot launch line, facing WEST. */
@Autonomous(name="Auto: Red Target", group="Auto", preselectTeleOp="TeleOp - Red")
public class Auto_Red_Target extends BaseAuto {
    @Override protected Alliance alliance() { return Alliance.RED; }
    @Override protected String startPoseDescription() { return "Start: Red Target — South depot launch line, FACING WEST"; }
    @Override protected boolean initialScanCW() { return true; } // CW first (toward goal)

    @Override
    protected void runSequence() throws InterruptedException {
        driveForwardInches(36.0);

        boolean locked = turnToGoalTag(2000);
        boolean atSpeed = locked && aimSpinUntilReady(3000);

        if (locked && atSpeed) {
            fireN(3);
        } else {
            telemetry.addLine("⚠️ No tag lock/at-speed — skipping volley");
            telemetry.update();
        }
        // Stay put.
    }
}
