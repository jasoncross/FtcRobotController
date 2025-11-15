package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.auto.BaseAuto.ScanDirection;
import org.firstinspires.ftc.teamcode.config.VisionTuning;

/*
 * FILE: Auto_Blue_Target.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/
 *
 * PURPOSE
 *   - Execute the BLUE alliance depot-side autonomous (south launch line,
 *     robot facing EAST) that rolls 36" to the tuned standoff, scans for Tag 20,
 *     delivers a five-artifact preload volley, then holds position to keep the
 *     depot lane clear.
 *   - Provide a concise AutoSequence reference for the depot route—drive to
 *     range, pre-spin, sweep, fire, and pause—so future tweaks can add scoring
 *     steps without rewriting the existing scaffold.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Autonomous pacing)
 *   - move(... 36.0 in, heading 0°, speed 0.55)
 *       • Sets the initial standoff before aiming. Power clamps via
 *         SharedRobotTuning.DRIVE_MAX_POWER for global speed changes.
 *   - spinToAutoRpmDefault(...)
 *       • Pre-spins the launcher with the shared AutoSpeed standby RPM so it’s
 *         ready before the tag sweep begins.
 *   - rotateToTarget(label, ScanDirection.CCW, turnSpeed 0.25, sweep 180°/-90°)
 *       • Sweeps counter-clockwise up to 180°, then backs clockwise to 90° shy of
 *         center before heading counter-clockwise again while searching for Tag 20.
 *   - readyToLaunch(timeout 3200 ms)
 *       • Waits for LauncherAutoSpeedController to reach the shared RPM window with settle gating.
 *   - fire(shots = 5, betweenShotsMs = 1000)
 *       • Commands the latest five-artifact preload with 1 s cadence between
 *         shots; adjust here if hardware needs more recovery time.
 *
 * METHODS
 *   - alliance()
 *       • Tags this routine as BLUE so BaseAuto uses Tag 20 lookups.
 *   - startPoseDescription()
 *       • Telemetry text confirming correct initial placement for the field crew.
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
    // CHANGES (2025-11-03): Renamed launcher prep steps to readyToLaunch()/spinToAutoRpmDefault() and
    //                        adopted the shared AutoSpeed settle behavior.
    // CHANGES (2025-11-13): Documented five-shot cadence + hold behavior in header for the refreshed depot plan.
    // BaseAuto needs the declared alliance to load the correct AprilTag IDs.
    @Override protected Alliance alliance() { return Alliance.BLUE; }
    // Telemetry annotation so setup crew knows correct orientation (edit to
    // update the Start Pose line shown on the Driver Station).
    @Override protected String startPoseDescription() { return "Start: Blue Target — South depot launch line, FACING EAST"; }

    @Override
    protected void runSequence() throws InterruptedException {
        sequence()
                .visionMode("Switch to 720p vision", VisionTuning.Mode.P720)
                .spinToAutoRpmDefault("Pre-spin launcher to auto RPM")
                .move("Drive 40 in to standoff", 40.0, 0.0, 0.55)
                // Telemetry label mirrors the shared driver callout; BaseAuto still targets the RED goal (ID 24).
                .rotateToTarget("Scan for Tag 24", ScanDirection.CCW, 0.25, 180, -90) // 180° CW sweep, CCW return to -90°, repeat
                .readyToLaunch("Ready launcher for volley", 3200)
                .fire("Fire volley", 5, true, 1000)
                //.waitFor("Hold position", 500)
                .run();
    }
}
