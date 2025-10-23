package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Alliance;

/*
 * START POSE:
 *   • Red Human: SOUTH firing triangle, EAST side, FACING NORTH.
 *   • Goal is to the LEFT/WEST → prefer CCW scan first.
 */
@Autonomous(name="Auto: Red Human", group="Auto", preselectTeleOp="TeleOp - Red")
public class Auto_Red_Human extends BaseAuto {
    @Override protected Alliance alliance() { return Alliance.RED; }
    @Override protected String startPoseDescription() {
        return "Red Human — South triangle (east side), facing NORTH";
    }
    @Override protected boolean initialScanCW() { return false; } // CCW first (left)

    @Override
    protected void runSequence() throws InterruptedException {
        double startHeading = drive.heading();
        turnToGoalTag(1500);
        aimSpinUntilReady(2500);
        fireN(3);
        // Human-side: return + drive forward
        turnBackTo(startHeading);
        driveForwardInches(24.0);
    }
}
