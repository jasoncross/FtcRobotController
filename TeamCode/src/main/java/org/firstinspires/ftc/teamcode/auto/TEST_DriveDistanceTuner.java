package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.odometry.FieldPose;

import java.util.Locale;

/*
 * FILE: TEST_DriveDistanceTuner.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/
 *
 * PURPOSE
 *   - Provide a dashboard-configurable AutoSequence harness that drives a single
 *     move or rotate per repetition so teams can tune encoder distance, heading
 *     offsets, and twist targets without editing code.
 *   - Surface clear telemetry about the requested distance, heading, twist, and
 *     speed cap while reusing BaseAuto safety telemetry and odometry updates.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Autonomous testing & diagnostics)
 *   - TEST_MODE / DISTANCE_IN / HEADING_DEG / TWIST_DEG / ROTATE_DEG
 *       • Selects the AutoSequence step to execute and the requested heading/
 *         twist offsets (or rotation delta) for that move.
 *   - SPEED_CAP
 *       • Limits translation/turn power for each test step while respecting the
 *         shared Auto drive caps inside BaseAuto.
 *   - REPS / SETTLE_MS / RESET_POSE_EACH_REP
 *       • Controls how many times the test repeats and how long to pause between
 *         repetitions, with optional odometry re-seeding each rep.
 *   - START_X / START_Y / START_HEADING
 *       • Defines the odometry seed pose used at init and optional per-rep reset.
 *
 * METHODS
 *   - alliance()
 *       • Returns BLUE so BaseAuto uses the default goal-tag mapping.
 *   - startPoseDescription()
 *       • Provides staging guidance for the distance-tuning test.
 *   - runSequence()
 *       • Executes one AutoSequence move or rotate per rep and reports poses.
 *
 * NOTES
 *   - Intended for practice-field tuning; no launcher, feed, or intake actions
 *     are triggered.
 */
@Config
@Autonomous(name="TEST: Drive Distance Tuner", group="TEST", preselectTeleOp="TeleOp - Blue")
public class TEST_DriveDistanceTuner extends BaseAuto {
    // CHANGES (2026-01-02): Added dashboard-driven auto test for move/rotate distance tuning.

    public static String TEST_NAME = "Drive Distance Tuner"; // Telemetry label for the current test profile.
    public static int REPS = 1; // Number of repetitions to run before finishing.
    public static int SETTLE_MS = 350; // Pause after each action to let the robot settle.
    public static double SPEED_CAP = 1.0; // Speed cap applied to each move or rotate action.
    public static double DISTANCE_IN = 48.0; // Travel distance for move-based test modes.
    public static double HEADING_DEG = 0.0; // Relative heading offset for CUSTOM moves.
    public static double TWIST_DEG = 0.0; // Twist target offset for MOVE_WITH_TWIST or CUSTOM.
    public static double ROTATE_DEG = 90.0; // Rotation delta for ROTATE_IN_PLACE tests.
    public static boolean RESET_POSE_EACH_REP = true; // Re-seed odometry to START_* before each rep.
    public static double START_X = 0.0; // Starting X pose in inches (field center frame).
    public static double START_Y = 0.0; // Starting Y pose in inches (field center frame).
    public static double START_HEADING = 0.0; // Starting heading in degrees (0 = field +Y).
    public static TestMode TEST_MODE = TestMode.FORWARD; // Selected test action executed each rep.

    public enum TestMode {
        FORWARD,
        BACKWARD,
        STRAFE_LEFT,
        STRAFE_RIGHT,
        DIAGONAL_45,
        DIAGONAL_NEG_45,
        ROTATE_IN_PLACE,
        MOVE_WITH_TWIST,
        CUSTOM
    }

    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }

    public TEST_DriveDistanceTuner() {
        setStartingPose(START_X, START_Y, START_HEADING);
    }

    @Override
    protected String startPoseDescription() {
        return "Start: Centered on tile, FACING NORTH (distance tuner)";
    }

    @Override
    protected void runSequence() throws InterruptedException {
        setStartingPose(START_X, START_Y, START_HEADING);
        for (int rep = 1; rep <= REPS && opModeIsActive(); rep++) {
            if (RESET_POSE_EACH_REP) {
                setStartingPose(START_X, START_Y, START_HEADING);
            }

            TestPlan plan = resolvePlan();
            String phase = String.format(Locale.US, "Rep %d/%d - %s", rep, REPS, plan.label);
            FieldPose prePose = updateOdometryPose();

            updateStatusWithPose(phase, false, prePose);
            telemetry.addData("Test Name", TEST_NAME);
            telemetry.addData("Test Mode", TEST_MODE);
            telemetry.addData("Rep", String.format(Locale.US, "%d/%d", rep, REPS));
            telemetry.addData("Distance (in)", String.format(Locale.US, "%.1f", DISTANCE_IN));
            telemetry.addData("Heading (deg)", String.format(Locale.US, "%.1f", plan.headingDeg));
            telemetry.addData("Twist (deg)", String.format(Locale.US, "%.1f", plan.twistDeg));
            telemetry.addData("Rotate (deg)", String.format(Locale.US, "%.1f", ROTATE_DEG));
            telemetry.addData("Speed Cap", String.format(Locale.US, "%.2f", SPEED_CAP));
            telemetry.update();

            if (plan.rotateInPlace) {
                sequence()
                        .rotate(phase, ROTATE_DEG, SPEED_CAP)
                        .run();
            } else {
                sequence()
                        .move(phase, DISTANCE_IN, plan.headingDeg, plan.twistDeg, SPEED_CAP)
                        .run();
            }

            if (!opModeIsActive()) {
                return;
            }

            sleep(SETTLE_MS);

            FieldPose postPose = updateOdometryPose();
            updateStatusWithPose(phase + " complete", false, postPose);
            telemetry.addData("Test Name", TEST_NAME);
            telemetry.addData("Test Mode", TEST_MODE);
            telemetry.addData("Rep", String.format(Locale.US, "%d/%d", rep, REPS));
            telemetry.addData("Pose (x,y,h)", String.format(Locale.US, "%.1f, %.1f, %.1f",
                    postPose.x, postPose.y, postPose.headingDeg));
            telemetry.update();
        }
    }

    private TestPlan resolvePlan() {
        switch (TEST_MODE) {
            case BACKWARD:
                return new TestPlan("Backward", 180.0, 0.0, false);
            case STRAFE_LEFT:
                return new TestPlan("Strafe Left", -90.0, 0.0, false);
            case STRAFE_RIGHT:
                return new TestPlan("Strafe Right", 90.0, 0.0, false);
            case DIAGONAL_45:
                return new TestPlan("Diagonal +45", 45.0, 0.0, false);
            case DIAGONAL_NEG_45:
                return new TestPlan("Diagonal -45", -45.0, 0.0, false);
            case ROTATE_IN_PLACE:
                return new TestPlan("Rotate In Place", 0.0, 0.0, true);
            case MOVE_WITH_TWIST:
                return new TestPlan("Move With Twist", 0.0, TWIST_DEG, false);
            case CUSTOM:
                return new TestPlan("Custom Move", HEADING_DEG, TWIST_DEG, false);
            case FORWARD:
            default:
                return new TestPlan("Forward", 0.0, 0.0, false);
        }
    }

    private static final class TestPlan {
        private final String label;
        private final double headingDeg;
        private final double twistDeg;
        private final boolean rotateInPlace;

        private TestPlan(String label, double headingDeg, double twistDeg, boolean rotateInPlace) {
            this.label = label;
            this.headingDeg = headingDeg;
            this.twistDeg = twistDeg;
            this.rotateInPlace = rotateInPlace;
        }
    }
}
