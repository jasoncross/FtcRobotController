package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Alliance;

/*
 * START POSE:
 *   • Blue Target: SOUTH edge of BLUE depot launch line, FACING EAST.
 *   • Goal is generally to the LEFT/NORTH from here → prefer CCW scan.
 *   • Requirement: drive ~36" in BEFORE aiming/firing; after volley, STAY PUT.
 */
@Autonomous(name="Auto: Blue Target", group="Auto", preselectTeleOp="TeleOp - Blue")
public class Auto_Blue_Target extends BaseAuto {
    @Override protected Alliance alliance() { return Alliance.BLUE; }
    @Override protected String startPoseDescription() {
        return "Blue Target — South depot launch line, facing EAST";
    }
    @Override protected boolean initialScanCW() { return false; } // CCW first (left)

    @Override
    protected void runSequence() throws InterruptedException {
        // Drive in before aiming (close to goal is inaccurate)
        driveForwardInches(36.0);

        // Lock, spin up, fire, then stay where we are
        turnToGoalTag(1500);
        aimSpinUntilReady(2500);
        fireN(3);

        // No turn-back or post-drive for Target-side per request
    }
}
