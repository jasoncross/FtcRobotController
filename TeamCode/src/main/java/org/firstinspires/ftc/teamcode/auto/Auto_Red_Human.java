package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Alliance;

/*
 * FILE: Auto_Red_Human.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/
 *
 * PURPOSE
 *   - Deliver the RED alliance human-player autonomous (east tile, robot facing
 *     NORTH) as documented in DECODE_Season_Context.md: spot Tag 24, confirm
 *     launcher readiness, fire the preload, and roll toward the classifier lane.
 *   - Provide a mirrored counterpart to Auto_Blue_Human so adjustments made to
 *     shared helpers behave identically on both alliances.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Autonomous pacing)
 *   - turnToGoalTag(2500 ms timeout)
 *       • Controls how long we look for AprilTag ID 24.
 *       • Shares the same SharedRobotTuning-driven twist cap and lock tolerance
 *         as the blue routine; extend toward 3200 ms if the red goal is darker.
 *   - aimSpinUntilReady(3000 ms timeout)
 *       • Waits for LauncherAutoSpeedController to reach the shared RPM window.
 *       • Works with SharedRobotTuning.SHOT_BETWEEN_MS for multi-ring cadence.
 *   - driveForwardInches(24.0)
 *       • Drives forward after the volley to open the intake lane.
 *       • Movement speed is bounded by SharedRobotTuning.DRIVE_MAX_POWER inside
 *         BaseAuto’s drive helpers; change that value for global adjustments.
 *
 * METHODS
 *   - alliance()
 *       • Identifies RED so BaseAuto requests Tag 24 and mirrors field geometry.
 *   - startPoseDescription()
 *       • Telemetry text ensuring the field crew stages the robot correctly.
 *   - initialScanCW()
 *       • Returns true because rotating clockwise (right turn) points the robot
 *         toward the red goal fastest from this start tile.
 *   - runSequence()
 *       • Captures the starting heading, performs the aim/spin checks, fires,
 *         then re-aligns and drives forward.
 *
 * NOTES
 *   - SharedRobotTuning updates propagate here automatically; alter the literal
 *     values only when the RED human strategy changes (e.g., different drive
 *     distance or follow-up task).
 */
@Autonomous(name="Auto: Red Human", group="Auto", preselectTeleOp="TeleOp - Red")
public class Auto_Red_Human extends BaseAuto {
    // Provide BaseAuto the active alliance to load correct AprilTag data.
    @Override protected Alliance alliance() { return Alliance.RED; }
    // Telemetry callout for the field-side volunteer verifying orientation.
    @Override protected String startPoseDescription() { return "Start: Red Human — East of south firing triangle, FACING NORTH"; }
    @Override protected boolean initialScanCW() { return true; } // CW first (turns right toward red goal)

    // Main autonomous path: aim, fire, and roll forward for cycle setup.
    @Override
    protected void runSequence() throws InterruptedException {
        double startHeading = drive.heading(); // Keep heading so we can unwind the turn

        boolean locked = turnToGoalTag(2500);               // Acquire AprilTag 24 within timeout
        boolean atSpeed = locked && aimSpinUntilReady(3000); // Wait for RPM only if tag lock succeeded

        if (locked && atSpeed) {
            fireN(3);
        } else {
            telemetry.addLine("⚠️ No tag lock/at-speed — skipping volley");
            telemetry.update();
        }

        turnBackTo(startHeading); // Return to original heading for the next path step
        driveForwardInches(24.0); // Advance toward intake/classifier lane
    }
}
