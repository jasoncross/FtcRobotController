package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Alliance;

/*
 * FILE: Auto_Red_Target.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/
 *
 * PURPOSE
 *   - Run the RED alliance depot-side autonomous (south launch line, robot
 *     facing WEST) that drives to a calibrated range, locks onto Tag 24, and
 *     fires the preload before parking to keep the lane open.
 *   - Mirrors Auto_Blue_Target so shared tuning behaves consistently across
 *     alliances.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Autonomous pacing)
 *   - driveForwardInches(36.0)
 *       • Sets the standoff distance before aiming.
 *       • BaseAuto handles power caps through SharedRobotTuning.DRIVE_MAX_POWER;
 *         edit that shared value for global speed tweaks.
 *   - turnToGoalTag(2500 ms timeout)
 *       • Time allowed to acquire AprilTag 24 after the 36" standoff drive.
 *       • Shares twist/tolerance limits with SharedRobotTuning; lengthen when
 *         post-drive vibrations slow down lock acquisition.
 *   - aimSpinUntilReady(3200 ms timeout)
 *       • Waits for AutoSpeed to reach the shared RPM window defined in
 *         SharedRobotTuning.RPM_TOLERANCE.
 *       • `fire(..., betweenShotsMs)` now accepts the cadence directly (3000 ms default here).
 *
 * METHODS
 *   - alliance()
 *       • Flags this routine as RED so BaseAuto looks for Tag 24 and mirrors
 *         field geometry.
 *   - startPoseDescription()
 *       • Telemetry reminder confirming start orientation for the setup crew.
 *   - initialScanCW()
 *       • Returns true so we rotate clockwise first—the fastest path to the red
 *         goal when starting facing WEST.
 *   - runSequence()
 *       • Drives forward, locks on, waits for RPM, fires, and intentionally holds.
 *
 * NOTES
 *   - Remaining stationary after the volley preserves the depot lane for the
 *     alliance partner; add extra movement only after confirming partner paths.
 */
@Autonomous(name="Auto: Red Target", group="Auto", preselectTeleOp="TeleOp - Red")
public class Auto_Red_Target extends BaseAuto {
    // CHANGES (2025-10-31): Matched refreshed spec – 36" drive, CW scan, gated volley, and
    //                        updated telemetry/hold behavior.
    // CHANGES (2025-10-31): Converted to AutoSequence for declarative standoff/aim/fire scripting
    //                        while preserving depot hold spacing.
    // CHANGES (2025-11-02): Added AutoSpeed pre-spin stage and explicit cadence parameter for volleys.
    // Provide BaseAuto with alliance context for mirrored helper logic.
    @Override protected Alliance alliance() { return Alliance.RED; }
    // Orientation reminder for match setup crew.
    @Override protected String startPoseDescription() { return "Start: Red Target — South depot launch line, FACING WEST"; }
    @Override protected boolean initialScanCW() { return true; } // CW first (turn right toward goal)

    @Override
    protected void runSequence() throws InterruptedException {
        sequence()
                .move("Drive 36 in to standoff", 36.0, 0.0, 0.55)
                .spinToAutoRpm("Pre-spin launcher to auto RPM")
                .rotateToTarget("Scan for Tag 24", 2500, true)
                .aim("Spin launcher for volley", 3200)
                .fire("Fire 3-shot volley", 3, true, 3000)
                .waitFor("Hold position", 500)
                .run();
    }
}
