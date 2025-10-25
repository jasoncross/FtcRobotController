package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Alliance;

/*
 * FILE: Auto_Blue_Target.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/
 *
 * PURPOSE
 *   - Execute the BLUE alliance depot-side autonomous (south launch line,
 *     robot facing EAST) that drives to a known standoff, locks onto AprilTag
 *     ID 20, and fires the preload before holding position.
 *   - Provide a clean example of combining driveForwardInches with the shared
 *     AutoAim helpers so students can bolt on additional scoring steps later.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Autonomous pacing)
 *   - driveForwardInches(36.0)
 *       • Sets the initial range prior to aiming.
 *       • BaseAuto.driveForwardInches() obeys SharedRobotTuning.DRIVE_MAX_POWER;
 *         edit that value when you need a global speed change.
 *   - turnToGoalTag(2000 ms timeout)
 *       • Allows the camera to settle on Tag 20.
 *       • Shares twist/tolerance caps with SharedRobotTuning just like other
 *         autos; extend the timeout if the robot drives farther before aiming.
 *   - aimSpinUntilReady(3000 ms timeout)
 *       • Waits for LauncherAutoSpeedController to reach the shared RPM window.
 *       • Works with SharedRobotTuning.SHOT_BETWEEN_MS for cadence once firing.
 *
 * METHODS
 *   - alliance()
 *       • Tags this routine as BLUE so BaseAuto uses Tag 20 lookups.
 *   - startPoseDescription()
 *       • Telemetry text confirming correct initial placement for the field crew.
 *   - initialScanCW()
 *       • Returns false so we sweep counter-clockwise first (facing EAST means
 *         the goal is to the robot’s left).
 *   - runSequence()
 *       • Drives to range, waits for aim/speed readiness, then fires.
 *
 * NOTES
 *   - Staying stationary post-volley protects the alliance partner’s intake path
 *     per drive strategy; add further movement steps only after confirming field
 *     spacing.
 */
@Autonomous(name="Auto: Blue Target", group="Auto", preselectTeleOp="TeleOp - Blue")
public class Auto_Blue_Target extends BaseAuto {
    // BaseAuto needs the declared alliance to load the correct AprilTag IDs.
    @Override protected Alliance alliance() { return Alliance.BLUE; }
    // Telemetry annotation so setup crew knows correct orientation.
    @Override protected String startPoseDescription() { return "Start: Blue Target — South depot launch line, FACING EAST"; }
    @Override protected boolean initialScanCW() { return false; } // CCW first (turn left toward goal)

    @Override
    protected void runSequence() throws InterruptedException {
        // Get some standoff from the tag; closer shots are inaccurate.
        driveForwardInches(36.0); // Adjust per TunableDirectory when range drift occurs

        boolean locked = turnToGoalTag(2000);              // Acquire Tag 20 with a modest timeout
        boolean atSpeed = locked && aimSpinUntilReady(3000); // Wait for RPM only if we saw the tag

        if (locked && atSpeed) {
            fireN(3);
        } else {
            telemetry.addLine("⚠️ No tag lock/at-speed — skipping volley");
            telemetry.update();
        }
        // Stay put (per request) to avoid interfering with partner.
    }
}
