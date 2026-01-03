package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.auto.BaseAuto.ScanDirection;
import org.firstinspires.ftc.teamcode.config.VisionTuning;

/*
 * FILE: Auto_Red_Human.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/
 *
 * PURPOSE
 *   - Execute the RED alliance human-player start (east tile, robot facing
 *     NORTH) by charging to the long-range shot box, sweeping for Tag 24,
 *     unleashing a rapid five-artifact volley, then retreating south to reopen
 *     the intake lane for the partner robot.
 *   - Mirror the BLUE alliance long-run routine so cross-alliance tuning stays
 *     synchronized while documenting the extended AutoSequence pattern of
 *     vision profile swap, heading capture, aggressive drive, and high-cadence
 *     firing.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Autonomous pacing)
 *   - visionMode(... Mode.P720)
 *       • Engages the 720p sighting profile before the long drive so range
 *         sampling matches TeleOp’s long-shot assumptions.
 *   - spinToAutoRpmDefault(...)
 *       • Keeps the launcher warm with the shared AutoSpeed default during the
 *         long sprint to the firing box.
 *   - move(... 80 in, heading 0°, twist 0°, speed 0.35)
 *       • Covers the full upfield sprint to the calibrated firing distance while
 *         holding the starting orientation prior to scanning for Tag 24.
 *   - rotateToTarget(label, ScanDirection.CW, turnSpeed 0.25, sweep 90°/30°, timeout 10000 ms)
 *       • Sweeps clockwise to 90° and checks 30° counter-clockwise while
 *         hunting for Tag 24; widen or shrink the arcs to tune scan coverage and
 *         raise/lower the timeout per route.
 *   - readyToLaunch(timeout 3200 ms)
 *       • Holds until AutoSpeed reaches the shared RPM window + settle timer so
 *         every shot leaves at the correct velocity.
 *   - fire(shots = 5, betweenShotsMs = 1000)
 *       • Fires a fast five-artifact volley once launcher readiness settles.
 *   - move(... -36 in, heading 0°, twist 0°, speed 0.85)
 *       • Drives 36" back toward the launch line to clear space for alliance
 *         partners immediately after shooting.
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
    // CHANGES (2025-11-03): Renamed launcher prep steps to readyToLaunch()/spinToAutoRpmDefault() and
    //                        adopted the shared AutoSpeed settle behavior.
    // CHANGES (2025-11-05): Added 720p vision profile swap at sequence start to mirror TeleOp testing.
    // CHANGES (2025-11-13): Updated header to describe long-run volley, five-shot cadence, and post-volley retreat.
    // CHANGES (2025-11-25): Inlined the 10 s rotate-to-target timeout directly on the scan step instead of using BaseAuto's default field.
    // CHANGES (2025-11-26): Standardized rotate-to-target timeout literal to 10000 ms for readability.
    // CHANGES (2025-11-24): Added explicit twist parameters (0°) to AutoSequence.move(...) calls per new API.
    // CHANGES (2025-12-11): Recentered odometry start pose to (+12, -72, 0) in the field-center frame (human wall = −72" Y).
    // Provide BaseAuto the active alliance to load correct AprilTag data.
    @Override protected Alliance alliance() { return Alliance.RED; }
    public Auto_Red_Human() { setStartingPose(12.0, -63.0, 0.0); }
    // Telemetry callout for the field-side volunteer verifying orientation (edit
    // this whenever start staging changes so the Start Pose telemetry stays
    // correct).
    @Override protected String startPoseDescription() { return "Start: Red Human — East of south firing triangle, FACING NORTH"; }

    // Main autonomous path: aim, fire, and roll forward for cycle setup.
    @Override
    protected void runSequence() throws InterruptedException {
        sequence()
                //.visionMode("Switch to 480p vision", VisionTuning.Mode.P480)
                .rememberHeading("Record start heading")
                .move("Drive forward to target firing zone", 80.0, 0.0, 0.0, 1)
                .rotateToTarget("Scan for Tag", ScanDirection.CW, 0.4, 90, 30, 10000)
                .readyToLaunch("Ready launcher for volley", 500)
                .fireContinuous("firing",1500,true, true)
               // .fire("Fire volley", 3, true, true, 0)
                .returnToStoredHeading("Return to start heading", 0.85)
                .move("Drive to balls", 58.0, -170.0, 90.0, 1)
                .move("Drive backward", 32.0, 180.0, 0.0, 1)
                .move("Drive to triangle", 58, 26.0, -115, 1)
                .rotateToTarget("Scan for Tag", ScanDirection.CW, 0.4, -30, 30, 1000)
                .readyToLaunch("Ready launcher for volley", 500)
                .fireContinuous("firing",1500,true, true)
                .move("Drive out",6,0,0,1)
                .run();
    }
}
