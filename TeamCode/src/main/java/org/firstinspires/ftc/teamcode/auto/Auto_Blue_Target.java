package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Alliance;

/** Blue Target — south edge of BLUE depot launch line, facing EAST. */
@Autonomous(name="Auto: Blue Target", group="Auto", preselectTeleOp="TeleOp - Blue")
public class Auto_Blue_Target extends BaseAuto {
    @Override protected Alliance alliance() { return Alliance.BLUE; }
    @Override protected String startPoseDescription() { return "Start: Blue Target — South depot launch line, FACING EAST"; }
    @Override protected boolean initialScanCW() { return false; } // CCW first (toward goal)

    @Override
    protected void runSequence() throws InterruptedException {
        // Get some standoff from the tag; closer shots are inaccurate.
        driveForwardInches(36.0);

        boolean locked = turnToGoalTag(2000);
        boolean atSpeed = locked && aimSpinUntilReady(3000);

        if (locked && atSpeed) {
            fireN(3);
        } else {
            telemetry.addLine("⚠️ No tag lock/at-speed — skipping volley");
            telemetry.update();
        }
        // Stay put (per request).
    }
}
