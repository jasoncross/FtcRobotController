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
 *   - turnToGoalTag(2000 ms timeout)
 *       • Time allowed to acquire AprilTag 24 after the drive.
 *       • Shares twist/tolerance limits with SharedRobotTuning; lengthen when
 *         post-drive vibrations slow down lock acquisition.
 *   - aimSpinUntilReady(3000 ms timeout)
 *       • Waits for AutoSpeed to reach the shared RPM window defined in
 *         SharedRobotTuning.RPM_TOLERANCE.
 *       • Cadence between shots is still governed by SharedRobotTuning.SHOT_BETWEEN_MS.
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
    // Provide BaseAuto with alliance context for mirrored helper logic.
    @Override protected Alliance alliance() { return Alliance.RED; }
    // Orientation reminder for match setup crew.
    @Override protected String startPoseDescription() { return "Start: Red Target — South depot launch line, FACING WEST"; }
    @Override protected boolean initialScanCW() { return true; } // CW first (turn right toward goal)

    @Override
    protected void runSequence() throws InterruptedException {
        driveForwardInches(36.0); // Shift to optimum range per driver testing

        boolean locked = turnToGoalTag(2000);              // Seek Tag 24 within timeout
        boolean atSpeed = locked && aimSpinUntilReady(3000); // Only wait for RPM if tag sighted

        if (locked && atSpeed) {
            fireN(3);
        } else {
            telemetry.addLine("⚠️ No tag lock/at-speed — skipping volley");
            telemetry.update();
        }
        // Stay put to avoid blocking partner autos.
    }
}
