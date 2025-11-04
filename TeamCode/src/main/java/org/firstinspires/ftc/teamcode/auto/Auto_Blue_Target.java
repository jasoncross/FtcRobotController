package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.auto.BaseAuto.ScanDirection;

/*
 * FILE: Auto_Blue_Target.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/
 *
 * PURPOSE
 *   - Execute the BLUE alliance depot-side autonomous (south launch line,
 *     robot facing EAST) that drives to a known standoff, locks onto AprilTag
 *     ID 20, and fires the preload before holding position.
 *   - Provide a clean example of chaining the AutoSequence builder’s move,
 *     rotate, aim, and fire steps so students can bolt on additional scoring
 *     actions later.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Autonomous pacing)
 *   - move(... 36.0 in, heading 0°, speed 0.55)
 *       • Sets the initial standoff before aiming. Power clamps via
 *         SharedRobotTuning.DRIVE_MAX_POWER for global speed changes.
 *   - rotateToTarget(label, ScanDirection.CCW, turnSpeed 0.25, sweep 180°/-90°)
 *       • Sweeps counter-clockwise up to 180°, then backs clockwise to 90° shy of
 *         center before heading counter-clockwise again while searching for Tag 20.
 *   - readyToLaunch(timeout 3200 ms)
 *       • Waits for LauncherAutoSpeedController to reach the shared RPM window with settle gating.
 *   - fire(shots = 3, betweenShotsMs = 3000)
 *       • Fires the preload volley with per-sequence cadence control.
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
    // BaseAuto needs the declared alliance to load the correct AprilTag IDs.
    @Override protected Alliance alliance() { return Alliance.BLUE; }
    // Telemetry annotation so setup crew knows correct orientation (edit to
    // update the Start Pose line shown on the Driver Station).
    @Override protected String startPoseDescription() { return "Start: Blue Target — South depot launch line, FACING EAST"; }

    @Override
    protected void runSequence() throws InterruptedException {
        sequence()
                .move("Drive 36 in to standoff", 36.0, 0.0, 0.55)
                .spinToAutoRpmDefault("Pre-spin launcher to auto RPM")
                .rotateToTarget("Scan for Tag 20", ScanDirection.CCW, 0.25, 180, -90) // 180° CCW sweep, CW return to +90°, repeat
                .readyToLaunch("Ready launcher for volley", 3200)
                .fire("Fire 3-shot volley", 3, true, 3000)
                .waitFor("Hold position", 500)
                .run();
    }
}
