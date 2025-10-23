package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Alliance;

/*
 * START POSE:
 *   • Red Target: SOUTH edge of RED depot launch line, FACING WEST.
 *   • Goal is generally to the RIGHT/NORTH from here → prefer CW scan.
 *   • Requirement: drive ~36" in BEFORE aiming/firing; after volley, STAY PUT.
 */
@Autonomous(name="Auto: Red Target", group="Auto", preselectTeleOp="TeleOp - Red")
public class Auto_Red_Target extends BaseAuto {
    @Override protected Alliance alliance() { return Alliance.RED; }
    @Override protected String startPoseDescription() {
        return "Red Target — South depot launch line, facing WEST";
    }
    @Override protected boolean initialScanCW() { return true; } // CW first (right)

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
