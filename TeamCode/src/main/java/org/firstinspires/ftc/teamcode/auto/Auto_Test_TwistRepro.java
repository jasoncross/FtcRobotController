package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Alliance;

/*
 * FILE: Auto_Test_TwistRepro.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/
 *
 * PURPOSE
 *   - Provide a focused autonomous repro that drives along field headings while
 *     twisting so crews can validate that moveWithTwist keeps translation
 *     field-centric during heading changes.
 *   - Establish a simple two-leg pattern (forward while twisting, then strafe
 *     while untwisting) to observe endpoint alignment after combined
 *     translation + rotation moves.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Autonomous pacing)
 *   - move("Field-forward + CCW twist", 48", heading 0°, twist +90°, speed 0.45)
 *       • Drives upfield while rotating counter-clockwise toward a 90° delta
 *         from the starting heading.
 *   - move("Field-right + CW twist back", 36", heading 90°, twist −90°, speed 0.45)
 *       • Strafes east while unwinding the previous twist back to the stored
 *         heading, verifying field-heading alignment during twist reversal.
 *
 * METHODS
 *   - alliance()
 *       • Uses BLUE as the default alliance for AprilTag selection while
 *         keeping the pattern alliance-agnostic for field testing.
 *   - startPoseDescription()
 *       • Telemetry reminder for where to stage the robot before running the
 *         repro.
 *   - runSequence()
 *       • Chains the twist-aware move() calls to expose the field-centric fix.
 *
 * NOTES
 *   - This OpMode is intended for validation on the practice field and shares
 *     all drive caps from SharedRobotTuning so it mirrors competition behavior.
 */
@Autonomous(name = "Test Auto: Twist Repro", group = "Auto", preselectTeleOp = "TeleOp - Blue")
public class Auto_Test_TwistRepro extends BaseAuto {
    // CHANGES (2025-12-13): New twist validation auto to confirm moveWithTwist keeps
    //                        translation field-centric while heading changes.
    // CHANGES (2025-12-13): Renamed to Auto_Test_TwistRepro and updated OpMode label to
    //                        clarify this is a test/diagnostic flow.

    @Override protected Alliance alliance() { return Alliance.BLUE; }
    public Auto_Test_TwistRepro() { setStartingPose(0.0, -60.0, 0.0); }
    @Override protected String startPoseDescription() { return "Start: Twist repro — Center line, FACING NORTH"; }

    @Override
    protected void runSequence() throws InterruptedException {
        sequence()
                .rememberHeading("Store starting heading")
                .move("Field-forward + CCW twist", 48.0, 0.0, 90.0, 0.45)
                .move("Field-right + CW twist back", 36.0, 90.0, -90.0, 0.45)
                .run();
    }
}

