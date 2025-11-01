package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Alliance;

/*
 * FILE: Auto_Blue_Human.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/
 *
 * PURPOSE
 *   - Run the BLUE alliance human-player side start (west tile, robot facing
 *     NORTH) exactly as described in DECODE_Season_Context.md so the preload is
 *     launched before transitioning toward the classifier zone.
 *   - Demonstrate how BaseAuto helpers (turnToGoalTag, aimSpinUntilReady,
 *     fireN) chain together for a straightforward "find Tag 20 → shoot → drive"
 *     routine that students can iterate on quickly.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Autonomous pacing)
 *   - turnToGoalTag(2600 ms timeout)
 *       • Timeout for acquiring AprilTag ID 20 after the 2" wall-clear bump.
 *       • Shares SharedRobotTuning.TURN_TWIST_CAP and LOCK_TOLERANCE_DEG for
 *         steering accuracy; extend toward 3200 ms when lighting is inconsistent.
 *   - aimSpinUntilReady(3200 ms timeout)
 *       • Waits for the launcher to reach SharedRobotTuning.RPM_TOLERANCE.
 *       • Shot cadence between volleys is governed globally by
 *         SharedRobotTuning.SHOT_BETWEEN_MS.
 *   - driveForwardInches(24.0)
 *       • Distance toward the classifier once the preload volley is finished.
 *       • Actual power cap comes from SharedRobotTuning.DRIVE_MAX_POWER inside
 *         BaseAuto.driveForwardInches(). Adjust distance here; adjust speed in
 *         the shared config.
 *
 * METHODS
 *   - alliance()
 *       • Identifies the BLUE alliance so BaseAuto selects Tag 20 by default.
 *   - startPoseDescription()
 *       • Telemetry helper used by drive teams to confirm the robot is staged
 *         correctly before autonomous starts.
 *   - initialScanCW()
 *       • Returns false so BaseAuto rotates counter-clockwise first—this points
 *         the bot toward the blue goal faster from the human station.
 *   - runSequence()
 *       • Main autonomous choreography: capture heading, aim, wait for RPM,
 *         fire, and reposition.
 *
 * NOTES
 *   - Any tuning to SharedRobotTuning propagates here immediately; only adjust
 *     the literal values in this file when the BLUE human route itself changes.
 *   - If TeleOpAllianceBase overrides AutoAim defaults (e.g., initialAutoDefaultSpeed),
 *     ensure BaseAuto.syncTeleOpOverrides() still mirrors those settings at init.
 */
@Autonomous(name="Auto: Blue Human", group="Auto", preselectTeleOp="TeleOp - Blue")
public class Auto_Blue_Human extends BaseAuto {
    // CHANGES (2025-10-31): Added wall-clear drive, telemetry-guided tag scan, locked volley,
    //                        heading reset, and 24" advance per refreshed Auto steps.
    // Alliance identity for BaseAuto scaffolding.
    @Override protected Alliance alliance() { return Alliance.BLUE; }
    // Telemetry label describing the expected robot orientation at init.
    @Override protected String startPoseDescription() { return "Start: Blue Human — West of south firing triangle, FACING NORTH"; }
    @Override protected boolean initialScanCW() { return false; } // CCW first (turns left toward blue goal)

    // Primary autonomous path: turn, confirm RPM, fire preload, reposition.
    @Override
    protected void runSequence() throws InterruptedException {
        double startHeading = drive.heading(); // Remember heading before any turn so we can return

        updateStatus("Clear wall (drive 2 in)", false);
        telemetry.update();
        driveForwardInches(2.0); // Clear the wall before scanning
        updateStatus("Scan setup", false);
        telemetry.update();

        boolean locked = turnToGoalTag(2600);               // Attempt to see AprilTag 20 within timeout
        boolean atSpeed = locked && aimSpinUntilReady(3200); // Only wait for RPM if we saw the tag

        if (locked && atSpeed) {
            updateStatus("Fire 3-shot volley", true);
            telemetry.update();
            fireN(3);
        } else {
            updateStatus("Hold position", false);
            telemetry.addLine("⚠️ No tag lock/at-speed — skipping volley");
            telemetry.update();
        }

        updateStatus("Return to start heading", locked && atSpeed);
        telemetry.update();
        turnBackTo(startHeading);       // Realign with original heading before driving forward
        updateStatus("Drive 24 in upfield", false);
        telemetry.update();
        driveForwardInches(24.0);       // Advance toward intake zone / classifier ramp
        updateStatus("Advance complete", false);
        telemetry.update();
    }
}
