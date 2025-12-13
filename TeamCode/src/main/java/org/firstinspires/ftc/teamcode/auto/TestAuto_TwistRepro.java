package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Alliance;

/*
 * FILE: TestAuto_TwistRepro.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/
 *
 * PURPOSE
 *   - Provide a contained regression harness that exercises AutoSequence.move(...)
 *     with and without twist offsets so drivetrain field-alignment can be validated
 *     after code changes.
 *   - Runs three short moves in sequence with telemetry labels only (no firing)
 *     to confirm the robot ends near the expected field coordinates while twisting
 *     to a new heading.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Autonomous pacing)
 *   - move("Baseline straight", 24", heading 0°, twist 0°, speed 0.45)
 *       • Confirms the standard straight-line move remains unchanged.
 *   - move("Heading 0 → twist +90", 24", heading 0°, twist +90°, speed 0.45)
 *       • Validates that forward translation holds the field heading while yawing
 *         toward +90°.
 *   - move("Heading 90 → twist +90", 24", heading 90°, twist +90°, speed 0.45)
 *       • Strafes right while twisting to face upfield for a combined strafe/turn
 *         check.
 *
 * METHODS
 *   - alliance()
 *       • Uses BLUE purely for telemetry consistency; no alliance-specific logic
 *         is invoked in this harness.
 *   - startPoseDescription()
 *       • Provides staging guidance for the drive team when running this test.
 *   - runSequence()
 *       • Chains the three reproduction moves with telemetry phase labels.
 *
 * NOTES
 *   - This OpMode is intended for practice-field validation only and does not
 *     manipulate intake, launcher, or feed mechanisms.
 */
@Autonomous(name="Test Auto: Twist Repro", group="Test")
public class TestAuto_TwistRepro extends BaseAuto {
    // CHANGES (2025-12-13): Added regression Auto to validate field-centric translation while twisting
    //                        through AutoSequence.move(...) with twist offsets applied.

    @Override
    protected Alliance alliance() { return Alliance.BLUE; }

    public TestAuto_TwistRepro() {
        setStartingPose(0.0, 0.0, 0.0);
    }

    @Override
    protected String startPoseDescription() {
        return "Start: Centered on tile, FACING NORTH (test harness)";
    }

    @Override
    protected void runSequence() throws InterruptedException {
        sequence()
                .move("Baseline straight", 24.0, 0.0, 0.0, 0.45)
                .move("Heading 0 → twist +90", 24.0, 0.0, 90.0, 0.45)
                .move("Heading 90 → twist +90", 24.0, 90.0, 90.0, 0.45)
                .run();
    }
}
