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
 *   - turnToGoalTag(2500 ms timeout)
 *       • Allows the camera to settle on Tag 20 while sweeping CCW.
 *       • Shares twist/tolerance caps with SharedRobotTuning just like other
 *         autos; extend the timeout if the robot drives farther before aiming.
 *   - aimSpinUntilReady(3200 ms timeout)
 *       • Waits for LauncherAutoSpeedController to reach the shared RPM window.
 *       • Followed by `fire(..., betweenShotsMs)` so cadence is chosen per sequence (defaults to 3000 ms here).
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
    // CHANGES (2025-10-31): Rebased routine on 36" standoff → CCW scan → locked volley with
    //                        shared telemetry + hold-in-place finish per Auto spec refresh.
    // CHANGES (2025-10-31): Adopted AutoSequence for easier route edits and no-lock volley
    //                        toggles while keeping depot hold behavior.
    // CHANGES (2025-11-02): Added pre-spin AutoSpeed warm-up and explicit volley cadence parameter.
    // BaseAuto needs the declared alliance to load the correct AprilTag IDs.
    @Override protected Alliance alliance() { return Alliance.BLUE; }
    // Telemetry annotation so setup crew knows correct orientation.
    @Override protected String startPoseDescription() { return "Start: Blue Target — South depot launch line, FACING EAST"; }
    @Override protected boolean initialScanCW() { return false; } // CCW first (turn left toward goal)

    @Override
    protected void runSequence() throws InterruptedException {
        sequence()
                .move("Drive 36 in to standoff", 36.0, 0.0, 0.55)
                .spinToAutoRpm("Pre-spin launcher to auto RPM")
                .rotateToTarget("Scan for Tag 20", 2500, false)
                .aim("Spin launcher for volley", 3200)
                .fire("Fire 3-shot volley", 3, true, 3000)
                .waitFor("Hold position", 500)
                .run();
    }
}
