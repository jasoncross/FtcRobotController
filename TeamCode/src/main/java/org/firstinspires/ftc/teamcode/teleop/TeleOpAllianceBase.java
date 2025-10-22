package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.drive.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Feed;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;

// === VISION IMPORTS ===
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.vision.VisionAprilTag;
import org.firstinspires.ftc.teamcode.vision.TagAimController;

import java.util.ArrayList;
import java.util.List;

// === CONTROLLER BINDINGS ===
import org.firstinspires.ftc.teamcode.input.ControllerBindings;
import static org.firstinspires.ftc.teamcode.input.ControllerBindings.Pad;
import static org.firstinspires.ftc.teamcode.input.ControllerBindings.Btn;
import static org.firstinspires.ftc.teamcode.input.ControllerBindings.Trigger;

/*
 * FILE: TeleOpAllianceBase.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleop/
 *
 * PURPOSE:
 *   Shared TeleOp base for both Red and Blue alliances.
 *   Manages driver input, drivetrain, launcher, feed, intake, and AprilTag-based auto-aim.
 *   Aim-Assist keeps the robot pointed at the alliance GOAL tag while preserving full translation control.
 *
 * CONTROLS (Gamepad 1):
 *   Left stick ........ Forward/back + strafe
 *   Right stick X ..... Rotation
 *   Left trigger ...... Brake (reduces top speed toward slowestSpeed)
 *   Right trigger ..... Manual launch RPM (only when manualSpeedMode == true)
 *   Left Bumper ....... Feed one ball (Feed subsystem)
 *   Right Bumper ...... Toggle intake ON/OFF
 *   Right Stick Btn ... TOGGLE Aim-Assist (not hold)
 *   Triangle / Y ...... Toggle manual launch-speed mode
 *
 * TELEMETRY (always shown):
 *   Alliance, BrakeCap, Intake state, ManualSpeed, RT value,
 *   RPM Target/Actual, Aim Enabled, Tag Visible, Tag Heading (deg),
 *   Tag Distance (inches).
 *
 * NOTES:
 *   - See ControllerBindings.java header for authoritative binding list.
 */

public abstract class TeleOpAllianceBase extends OpMode {
    protected abstract Alliance alliance();

    // ---------------- Subsystems ----------------
    protected Drivebase drive;
    protected Launcher launcher;
    protected Feed feed;
    protected Intake intake;

    // ---------------- Tunables ----------------
    private double slowestSpeed = 0.25;
    private double rpmBottom    = 0;
    private double rpmTop       = 6000;

    // ---------------- State ----------------
    private boolean manualSpeedMode = true;
    private boolean aimEnabled = false;

    // ---------------- Vision + Aim ----------------
    private VisionAprilTag vision;
    private TagAimController aim = new TagAimController();

    // ---------------- Pose Smoothing ----------------
    private Double smHeadingDeg = null;
    private Double smRangeMeters = null;
    private static final double SMOOTH_A = 0.25;
    private static final double M_TO_IN = 39.37007874015748;

    // ---------------- Controller Bindings ----------------
    private ControllerBindings controls;


// ---------------- RPM Test Mode ----------------
// FILE: TeleOpAllianceBase.java
// PURPOSE: Allow bench-testing launcher RPM via D-pad (Up/Down/Left/Right).
private boolean rpmTestEnabled = false;
private double  rpmTestTarget  = 0.0;

private static final double RPM_TEST_STEP = 50.0;   // increment per Left/Right press
private static final double RPM_TEST_MIN  = 0.0;
private static final double RPM_TEST_MAX  = 6000.0;

// Local clamp helper (keeps RPM in range)
private static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
}

@Override
    public void init() {
        // ---- Subsystem Initialization ----
        drive    = new Drivebase(hardwareMap, telemetry);
        launcher = new Launcher(hardwareMap);
        feed     = new Feed(hardwareMap);
        intake   = new Intake(hardwareMap);

        // ---- Vision Initialization ----
        vision = new VisionAprilTag();
        vision.init(hardwareMap, "Webcam 1");
        vision.setRangeScale(0.03);

        // ---- Controller Bindings Setup ----
        controls = new ControllerBindings();

        /*
         * GAMEPAD 1 (Driver)
         *   LB  -> Feed once
         *   RB  -> Intake toggle
         *   RS  -> Aim-assist toggle
         *   Y   -> Manual-speed mode toggle
         *   RT  -> Manual RPM set (axis) [G1 ONLY]
         */
        controls
            .bindPress(Pad.G1, Btn.LB, () -> feed.feedOnceBlocking())
            .bindPress(Pad.G1, Btn.RB, () -> intake.toggle())
            .bindToggle(Pad.G1, Btn.R_STICK_BTN,
                () -> { aimEnabled = true;  },
                () -> { aimEnabled = false; })
            .bindToggle(Pad.G1, Btn.Y,
                () -> { manualSpeedMode = true;  },
                () -> { manualSpeedMode = false; })
            .bindTriggerAxis(Pad.G1, Trigger.RT, (rt0to1) -> {
                if (manualSpeedMode) {
                    double target = rpmBottom + rt0to1 * (rpmTop - rpmBottom);
                    launcher.setTargetRpm(target);
                }
            });

        /*
         * GAMEPAD 2 (Co-Driver) — Same functions, EXCEPT anything involving joysticks:
         *   LB -> Feed once
         *   RB -> Intake toggle
         *   Y  -> Manual-speed mode toggle
         */
        controls
            .bindPress(Pad.G2, Btn.LB, () -> feed.feedOnceBlocking())
            .bindPress(Pad.G2, Btn.RB, () -> intake.toggle())
            .bindToggle(Pad.G2, Btn.Y,
                () -> { manualSpeedMode = true;  },
                () -> { manualSpeedMode = false; });

        telemetry.addData("TeleOp", "Alliance: %s", alliance());
        telemetry.addLine("Bindings: See ControllerBindings.java for bound functions.");
        telemetry.update();


})
.bindPress(ControllerBindings.Pad.G1, ControllerBindings.Btn.DPAD_RIGHT, () -> {
    if (rpmTestEnabled) rpmTestTarget = clamp(rpmTestTarget + RPM_TEST_STEP, RPM_TEST_MIN, RPM_TEST_MAX);
})
// Down: disable test mode and stop launcher
.bindPress(ControllerBindings.Pad.G1, ControllerBindings.Btn.DPAD_DOWN, () -> {
    rpmTestEnabled = false;
    launcher.stop();
});

    }

    @Override
    public void loop() {
        // ==============================================================
        // UPDATE CONTROLLER BINDINGS FIRST
        // ==============================================================
        controls.update(gamepad1, gamepad2);


        // RPM Test Mode override
        if (rpmTestEnabled) { launcher.setTargetRpm(rpmTestTarget); }

        // ==============================================================
        // DRIVETRAIN CONTROL
        // ==============================================================
        double brake = gamepad1.left_trigger;
        double cap = 1.0 - brake * (1.0 - slowestSpeed);

        double driveY  = cap * gamepad1.left_stick_y;
        double strafeX = cap * -gamepad1.left_stick_x;
        double twist   = cap * -gamepad1.right_stick_x;

        // ==============================================================
        // ALLIANCE TARGET TAG SELECTION
        // ==============================================================
        int targetId = (alliance() == Alliance.BLUE)
                ? VisionAprilTag.TAG_BLUE_GOAL
                : VisionAprilTag.TAG_RED_GOAL;

        // ==============================================================
        // APRILTAG DETECTION + SMOOTHING + AIM OVERRIDE
        // ==============================================================
        AprilTagDetection goalDet = vision.getDetectionFor(targetId);

        if (goalDet != null) {
            double h = goalDet.ftcPose.bearing;
            double r = goalDet.ftcPose.range;
            smHeadingDeg  = (smHeadingDeg  == null) ? h : (SMOOTH_A * h + (1 - SMOOTH_A) * smHeadingDeg);
            smRangeMeters = (smRangeMeters == null) ? r : (SMOOTH_A * r + (1 - SMOOTH_A) * smRangeMeters);
        }

        if (aimEnabled && goalDet != null) {
            twist = aim.turnPower(goalDet);
        }

        // ==============================================================
        // DRIVEBASE EXECUTION
        // ==============================================================
        drive.drive(driveY, strafeX, twist);

        // ==============================================================
        // TELEMETRY OUTPUT (original + vision)
        // ==============================================================
        telemetry.addData("Alliance", alliance());
        telemetry.addData("BrakeCap", "%.2f", cap);
        telemetry.addData("Intake", intake.isOn() ? "On" : "Off");
        telemetry.addData("ManualSpeed", manualSpeedMode);
        telemetry.addData("RT", "%.2f", gamepad1.right_trigger);
        telemetry.addData("RPM Target", "%.0f", launcher.targetRpm);
        telemetry.addData("RPM Actual", "%.0f", launcher.getCurrentRpm());

        telemetry.addData("Aim Enabled", aimEnabled);
        telemetry.addData("Target Tag ID", targetId);

        List<Integer> seenIds = new ArrayList<>();
        for (AprilTagDetection d : vision.getDetectionsCompat()) seenIds.add(d.id);
        telemetry.addData("Seen Tags (count)", seenIds.size());
        telemetry.addData("Seen Tag IDs", seenIds.isEmpty() ? "[]" : seenIds.toString());

        double headingDeg = (smHeadingDeg == null) ? Double.NaN : smHeadingDeg;
        double distM = vision.getScaledRange(goalDet);
        double distIn = Double.isNaN(distM) ? Double.NaN : distM * 39.3701;

        telemetry.addData("Goal Tag Visible", goalDet != null);
        telemetry.addData("Goal Heading (deg)", Double.isNaN(headingDeg) ? "---" : String.format("%.1f", headingDeg));
        telemetry.addData("Tag Distance (in)", Double.isNaN(distIn) ? "---" : String.format("%.1f", distIn));

        if (rpmTestEnabled) telemetry.addData("RPM Test", "ENABLED");
        if (rpmTestEnabled) telemetry.addData("RPM Test Target", "%.0f", rpmTestTarget);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (vision != null) vision.stop();
    }
}
