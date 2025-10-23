package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Alliance;

/** Red Human — east side of south firing triangle, facing NORTH. */
@Autonomous(name="Auto: Red Human", group="Auto", preselectTeleOp="TeleOp - Red")
public class Auto_Red_Human extends BaseAuto {
    @Override protected Alliance alliance() { return Alliance.RED; }
    @Override protected String startPoseDescription() { return "Start: Red Human — East of south firing triangle, FACING NORTH"; }
    @Override protected boolean initialScanCW() { return true; } // CW first

    @Override
    protected void runSequence() throws InterruptedException {
        double startHeading = drive.heading();

        boolean locked = turnToGoalTag(2500);
        boolean atSpeed = locked && aimSpinUntilReady(3000);

        if (locked && atSpeed) {
            fireN(3);
        } else {
            telemetry.addLine("⚠️ No tag lock/at-speed — skipping volley");
            telemetry.update();
        }

        turnBackTo(startHeading);
        driveForwardInches(24.0);
    }
}
