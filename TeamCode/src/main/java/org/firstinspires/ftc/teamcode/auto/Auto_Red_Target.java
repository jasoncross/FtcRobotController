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
 *   - move(... 36 in, heading 0°, twist 0°, speed 0.55)
 *       • Sets the standoff distance before aiming while holding the starting
 *         heading. Power clamps through SharedRobotTuning.DRIVE_MAX_POWER for
 *         global tweaks.
 *   - spinToAutoRpmDefault(...)
 *       • Pre-spins the launcher so Tag 24 sweeps start with the wheels already
 *         at the standby AutoSpeed RPM.
 *   - rotateToTarget(label, ScanDirection.CW, turnSpeed 0.25, sweep 180°/-90°, timeout 10000 ms)
 *       • Sweeps clockwise up to 180°, then backs counter-clockwise to 90° shy of
 *         center before heading clockwise again while searching for Tag 24; adjust
 *         the timeout per route when changing search strategy.
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
    // CHANGES (2025-11-25): rotateToTarget scan now hard-codes the 10 s timeout on the call instead of relying on BaseAuto.
    // CHANGES (2025-11-26): Standardized rotate-to-target timeout literal to 10000 ms for readability.
    // CHANGES (2025-11-24): Added explicit twist parameter (0°) to AutoSequence.move(...) per new API.
    // CHANGES (2026-01-10): Added AutoSequence AutoRPM tweak step for TeleOp-style scale adjustments.
    // Provide BaseAuto with alliance context for mirrored helper logic.
    @Override protected Alliance alliance() { return Alliance.RED; }
    // Orientation reminder for match setup crew (edit to refresh the Start Pose
    // telemetry string whenever placement changes).
    @Override protected String startPoseDescription() { return "Start: Red Target — South depot launch line, FACING WEST"; }

    @Override
    protected void runSequence() throws InterruptedException {
        sequence()
                .adjustAutoScale("AutoRPM tweak +2%", 0.02)
                //.visionMode("Switch to 480p vision", VisionTuning.Mode.P480)
                .rememberHeading("Record start heading")
                .move("Drive 40 in to standoff", 40.0, 0.0, 0.0, 1)
                .rotateToTarget("Scan for Tag 24", ScanDirection.CW, 0.45, 180, -90, 10000) // 180° CW sweep, CCW return to -90°, repeat
                .readyToLaunch("Ready launcher for volley", 500)
                .fireContinuous("firing",2000,true, true)
                .returnToStoredHeading("Return to start heading", 0.45)
                .move("Drive to balls", 26.0, 90.0, -45.0, 1)
                .move("Drive backward", 26.0, 180.0, 0.0, 1)
                .move("Drive to triangle", 48.0, -15.0, -135.0, 1)
                //.fire("Fire volley", 3, false, true, 0)
                .readyToLaunch("Ready launcher for volley", 500)
                .fireContinuous("firing",2000,true, true)
                //.waitFor("Hold position", 500)
                .run();
    }
}
