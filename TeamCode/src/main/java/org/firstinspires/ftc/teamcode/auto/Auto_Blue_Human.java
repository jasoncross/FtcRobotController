package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Alliance;

/*
 * START POSE (from field diagram):
 *   • Blue Human: SOUTH firing triangle, WEST side, FACING NORTH.
 *   • Goal is to the RIGHT/EAST → prefer CW scan first.
 */
@Autonomous(name="Auto: Blue Human", group="Auto", preselectTeleOp="TeleOp - Blue")
public class Auto_Blue_Human extends BaseAuto {
    @Override protected Alliance alliance() { return Alliance.BLUE; }
    @Override protected String startPoseDescription() {
        return "Blue Human — South triangle (west side), facing NORTH";
    }
    @Override protected boolean initialScanCW() { return true; } // CW first (right)

    @Override
    protected void runSequence() throws InterruptedException {
        double startHeading = drive.heading();
        turnToGoalTag(1500);
        aimSpinUntilReady(2500);
        fireN(3);
        // Human-side: return to start heading + drive upfield 24"
        turnBackTo(startHeading);
        driveForwardInches(24.0);
    }
}
