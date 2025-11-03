package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.auto.BaseAuto.ScanDirection;

/*
 * FILE: Auto_Red_Human.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/
 *
 * PURPOSE
 *   - Deliver the RED alliance human-player autonomous (east tile, robot facing
 *     NORTH) as documented in DECODE_Season_Context.md: spot Tag 24, confirm
 *     launcher readiness, fire the preload, and roll toward the classifier lane.
 *   - Provide a mirrored counterpart to Auto_Blue_Human so adjustments made to
 *     shared helpers behave identically on both alliances while showcasing the
 *     AutoSequence flow.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Autonomous pacing)
 *   - move(... 2 in, heading 0°, speed 0.35)
 *       • Soft bump off the wall before scanning for Tag 24.
 *   - rotateToTarget(label, ScanDirection.CW, turnSpeed 0.25, sweep 90°/30°)
 *       • Sweeps clockwise up to 90° after the wall-clear, then checks 30°
 *         counter-clockwise while searching for Tag 24. Increase sweep angles
 *         or the speed fraction for wider hunts.
 *   - aim(timeout 3200 ms)
 *       • Waits for LauncherAutoSpeedController to reach the shared RPM window.
 *   - fire(shots = 3, betweenShotsMs = 3000)
 *       • Encodes cadence inline (3000 ms default here).
 *   - move(... 24 in, heading 0°, speed 0.55)
 *       • Drives forward after the volley to open the intake lane.
 *
 * METHODS
 *   - alliance()
 *       • Identifies RED so BaseAuto requests Tag 24 and mirrors field geometry.
 *   - startPoseDescription()
 *       • Telemetry text ensuring the field crew stages the robot correctly.
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
    // CHANGES (2025-10-31): Added wall-clear bump, telemetry-guided Tag 24 volley, heading
    //                        reset, and 24" advance mirroring the refreshed Auto flow.
    // CHANGES (2025-10-31): Migrated to AutoSequence builder for declarative steps and
    //                        clarified lock/aim/fire sequencing with configurable speed caps.
    // CHANGES (2025-11-02): Added AutoSpeed warm-up stage and explicit volley spacing parameter.
    // Provide BaseAuto the active alliance to load correct AprilTag data.
    @Override protected Alliance alliance() { return Alliance.RED; }
    // Telemetry callout for the field-side volunteer verifying orientation (edit
    // this whenever start staging changes so the Start Pose telemetry stays
    // correct).
    @Override protected String startPoseDescription() { return "Start: Red Human — East of south firing triangle, FACING NORTH"; }

    // Main autonomous path: aim, fire, and roll forward for cycle setup.
    @Override
    protected void runSequence() throws InterruptedException {
        sequence()
                .rememberHeading("Record start heading")
                .move("Clear wall (drive 2 in)", 2.0, 0.0, 0.35)
                .spinToAutoRpm("Pre-spin launcher to auto RPM")
                .rotateToTarget("Scan for Tag 24", ScanDirection.CW, 0.25, 90, 30)
                .aim("Spin launcher for volley", 3200)
                .fire("Fire 3-shot volley", 3, true, 3000)
                .returnToStoredHeading("Return to start heading", 0.45)
                .move("Drive 24 in upfield", 24.0, 0.0, 0.55)
                .run();
    }
}
