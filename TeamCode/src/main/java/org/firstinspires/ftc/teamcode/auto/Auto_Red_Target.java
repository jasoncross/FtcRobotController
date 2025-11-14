package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.auto.BaseAuto.ScanDirection;
import org.firstinspires.ftc.teamcode.config.VisionTuning;

/*
 * FILE: Auto_Red_Target.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/
 *
 * PURPOSE
 *   - Execute the RED alliance depot-side autonomous (south launch line, robot
 *     facing WEST) that advances 36" to the tuned standoff, sweeps for Tag 24,
 *     unloads the full five-artifact preload, and parks in place to keep the
 *     depot lane clear.
 *   - Mirror the BLUE depot routine so shared tuning stays synchronized while
 *     documenting the latest five-shot cadence and hold-in-place finish for the
 *     RED side.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Autonomous pacing)
 *   - move(... 36 in, heading 0°, speed 0.55)
 *       • Sets the standoff distance before aiming. Power clamps through
 *         SharedRobotTuning.DRIVE_MAX_POWER for global tweaks.
 *   - spinToAutoRpmDefault(...)
 *       • Pre-spins the launcher so Tag 24 sweeps start with the wheels already
 *         at the standby AutoSpeed RPM.
 *   - rotateToTarget(label, ScanDirection.CW, turnSpeed 0.25, sweep 180°/-90°)
 *       • Sweeps clockwise up to 180°, then backs counter-clockwise to 90° shy of
 *         center before heading clockwise again while searching for Tag 24.
 *   - readyToLaunch(timeout 3200 ms)
 *       • Waits for AutoSpeed to reach the shared RPM window + settle time defined in
 *         SharedRobotTuning.
 *   - fire(shots = 5, betweenShotsMs = 1000)
 *       • Executes the refreshed five-artifact preload volley with 1 s cadence
 *         between shots.
 *
 * METHODS
 *   - alliance()
 *       • Flags this routine as RED so BaseAuto looks for Tag 24 and mirrors
 *         field geometry.
 *   - startPoseDescription()
 *       • Telemetry reminder confirming start orientation for the setup crew.
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
    // CHANGES (2025-11-03): Renamed launcher prep steps to readyToLaunch()/spinToAutoRpmDefault() and
    //                        adopted the shared AutoSpeed settle behavior.
    // CHANGES (2025-11-13): Updated header to capture five-shot volley and depot hold notes for RED side.
    // Provide BaseAuto with alliance context for mirrored helper logic.
    @Override protected Alliance alliance() { return Alliance.RED; }
    // Orientation reminder for match setup crew (edit to refresh the Start Pose
    // telemetry string whenever placement changes).
    @Override protected String startPoseDescription() { return "Start: Red Target — South depot launch line, FACING WEST"; }

    @Override
    protected void runSequence() throws InterruptedException {
        sequence()
                .visionMode("Switch to 720p vision", VisionTuning.Mode.P720)
                .spinToAutoRpmDefault("Pre-spin launcher to auto RPM")
                .move("Drive 36 in to standoff", 36.0, 0.0, 0.55)
                // Telemetry label mirrors the shared driver callout; BaseAuto still targets the RED goal (ID 24).
                .rotateToTarget("Scan for Tag 24", ScanDirection.CW, 0.25, 180, -90) // 180° CW sweep, CCW return to -90°, repeat
                .readyToLaunch("Ready launcher for volley", 3200)
                .fire("Fire volley", 5, true, 1000)
                //.waitFor("Hold position", 500)
                .run();
    }
}
