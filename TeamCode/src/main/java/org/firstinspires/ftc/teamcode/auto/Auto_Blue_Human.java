package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Alliance;

/** Blue Human — west side of south firing triangle, facing NORTH. */
@Autonomous(name="Auto: Blue Human", group="Auto", preselectTeleOp="TeleOp - Blue")
public class Auto_Blue_Human extends BaseAuto {
    @Override protected Alliance alliance() { return Alliance.BLUE; }
    @Override protected String startPoseDescription() { return "Start: Blue Human — West of south firing triangle, FACING NORTH"; }
    @Override protected boolean initialScanCW() { return false; } // CCW first

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
