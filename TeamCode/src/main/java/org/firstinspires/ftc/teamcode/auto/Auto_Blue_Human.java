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
 *   - turnToGoalTag(2500 ms timeout)
 *       • Timeout for acquiring AprilTag ID 20.
 *       • Shares SharedRobotTuning.TURN_TWIST_CAP and LOCK_TOLERANCE_DEG for
 *         steering accuracy; extend toward 3200 ms when lighting is inconsistent.
 *   - aimSpinUntilReady(3000 ms timeout)
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
    // Alliance identity for BaseAuto scaffolding.
    @Override protected Alliance alliance() { return Alliance.BLUE; }
    // Telemetry label describing the expected robot orientation at init.
    @Override protected String startPoseDescription() { return "Start: Blue Human — West of south firing triangle, FACING NORTH"; }
    @Override protected boolean initialScanCW() { return false; } // CCW first (turns left toward blue goal)

    // Primary autonomous path: turn, confirm RPM, fire preload, reposition.
    @Override
    protected void runSequence() throws InterruptedException {
        double startHeading = drive.heading(); // Remember heading before any turn so we can return

        boolean locked = turnToGoalTag(2500);              // Attempt to see AprilTag 20 within timeout
        boolean atSpeed = locked && aimSpinUntilReady(3000); // Only wait for RPM if we saw the tag

        if (locked && atSpeed) {
            fireN(3);
        } else {
            telemetry.addLine("⚠️ No tag lock/at-speed — skipping volley");
            telemetry.update();
        }

        turnBackTo(startHeading);       // Realign with original heading before driving forward
        driveForwardInches(24.0);       // Advance toward intake zone / classifier ramp
    }
}
