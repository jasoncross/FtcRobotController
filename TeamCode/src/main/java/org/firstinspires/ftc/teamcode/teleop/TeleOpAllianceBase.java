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
 *   X / A ............. Feed one ball (Feed subsystem)
 *   Left Bumper ....... Toggle intake ON/OFF
 *   Right Bumper ...... TOGGLE Aim-Assist (not hold)
 *   Triangle / Y ...... Toggle manual launch-speed mode
 *
 * TELEMETRY (always shown):
 *   Alliance, BrakeCap, Intake state, ManualSpeed, RT value,
 *   RPM Target/Actual, Aim Enabled, Tag Visible, Tag Heading (deg),
 *   Tag Distance (inches).
 *
 * TUNABLES:
 *   // Drivebase (see Drivebase.java)
 *   - STRAFE_CORRECTION ............... Multiplier to linearize strafe distance
 *   - TURN_KP, TURN_KD ............... IMU turn gains
 *   - TURN_TOLERANCE_DEG, TURN_SETTLE_TIME ... Turn completion criteria
 *
 *   // TeleOp behavior
 *   - slowestSpeed .................... Full-brake top-speed cap (0–1)
 *   - rpmBottom, rpmTop .............. Manual launch RPM range mapped from RT when manualSpeedMode == true
 *
 *   // Vision / Aim (see vision/)
 *   - TagAimController.kP ............ Proportional gain for bearing error (default 0.02)
 *   - TagAimController.kD ............ Damping gain (default 0.003)
 *   - TagAimController twist clamp ... Max ±0.6 twist power to preserve driver control while translating
 *   - TagAimController deadband ...... ±1.5° stop band to prevent hunting
 *   - VisionAprilTag.rangeScale ...... Distance scale factor (default 1.0); set via one-point calibration
 *   - VisionAprilTag.cameraResolution . 640×480 (uses built-in calibration)
 *   - VisionAprilTag.streamFormat .... MJPEG (higher FPS; ignored if unsupported)
 *
 * DISTANCE & UNITS:
 *   - VisionAprilTag reports scaled range (meters) which TeleOp converts to inches (m × 39.3701) for display.
 *
 * NOTES:
 *   - Webcam must be added to Robot Configuration as "Webcam 1".
 *   - IMU physical orientation assumed: Logo UP, USB RIGHT.
 *   - This file preserves all existing telemetry and subsystem logic; vision telemetry is appended.
 */


public abstract class TeleOpAllianceBase extends OpMode {
    // Alliance is implemented by TeleOp_Red / TeleOp_Blue
    protected abstract Alliance alliance();

    // ---------------- Subsystems ----------------
    protected Drivebase drive;
    protected Launcher launcher;
    protected Feed feed;
    protected Intake intake;

    // ---------------- Tunables ----------------
    private double slowestSpeed = 0.25;  // Brake cap
    private double rpmBottom    = 0;     // Manual RPM bottom range
    private double rpmTop       = 6000;  // Manual RPM top range

    // ---------------- State ----------------
    private boolean manualSpeedMode = true; // Triangle toggles this

    // Button edge tracking
    private boolean prevY  = false; // Triangle
    private boolean prevLB = false; // Left Bumper (Intake)
    private boolean prevA  = false; // X/A (Feed)
    private boolean prevRB = false; // Right Bumper (Aim toggle)

    // ---------------- Vision + Aim ----------------
    private VisionAprilTag vision;
    private TagAimController aim = new TagAimController();
    private boolean aimEnabled = false;  // Toggled via Right Bumper

    // ---------------- Pose Smoothing (simple EMA) ----------------
    private Double smHeadingDeg = null;       // smoothed bearing (deg)
    private Double smRangeMeters = null;      // smoothed distance (m)
    private static final double SMOOTH_A = 0.25; // 0..1 (higher=snappier, lower=smoother)

    // ---------------- Units ----------------
    private static final double M_TO_IN = 39.37007874015748; // meters → inches

    @Override
    public void init() {
        // ---- Subsystem Initialization ----
        drive    = new Drivebase(hardwareMap, telemetry);
        launcher = new Launcher(hardwareMap);
        feed     = new Feed(hardwareMap);
        intake   = new Intake(hardwareMap);

        // ---- Vision Initialization ----
        vision = new VisionAprilTag();
        vision.init(hardwareMap, "Webcam 1"); // Must match Robot Configuration
        vision.setRangeScale(0.03); // <-- your calibrated value


        telemetry.addData("TeleOp", "Alliance: %s", alliance());
        telemetry.addLine("Aim: Right Bumper toggles ON/OFF");
        telemetry.update();
    }

    @Override
    public void loop() {
        // ==============================================================
        // DRIVETRAIN CONTROL
        // ==============================================================
        double brake = gamepad1.left_trigger; // 0..1
        double cap = 1.0 - brake * (1.0 - slowestSpeed);

        double driveY  = cap * gamepad1.left_stick_y;   // Forward/back
        double strafeX = cap * -gamepad1.left_stick_x;  // Left/right
        double twist   = cap * -gamepad1.right_stick_x; // Rotation

        // ==============================================================
        // LAUNCHER: RIGHT TRIGGER → MANUAL RPM (WHEN ENABLED)
        // ==============================================================
        if (manualSpeedMode) {
            double rt = gamepad1.right_trigger; // 0..1
            double target = rpmBottom + rt * (rpmTop - rpmBottom);
            launcher.setTargetRpm(target);
        }

        // ==============================================================
        // AIM TOGGLE (Right Bumper)
        // ==============================================================
        boolean rb = gamepad1.right_bumper;
        if (rb && !prevRB) {
            aimEnabled = !aimEnabled;
        }
        prevRB = rb;

        // ==============================================================
        // ALLIANCE TARGET TAG SELECTION
        // ==============================================================
        int targetId = (alliance() == Alliance.BLUE)
                ? VisionAprilTag.TAG_BLUE_GOAL   // 20
                : VisionAprilTag.TAG_RED_GOAL;   // 24

        // ==============================================================
        // APRILTAG DETECTION + SMOOTHING + AIM OVERRIDE
        // - We ALWAYS read detection list (for telemetry).
        // - We ONLY override twist when Aim is enabled *and* the goal tag is visible.
        // ==============================================================
        AprilTagDetection goalDet = vision.getDetectionFor(targetId);

        // Smooth heading/range for steadier telemetry (and steadier aim if desired)
        if (goalDet != null) {
            double h = goalDet.ftcPose.bearing; // degrees
            double r = goalDet.ftcPose.range;   // meters (already scaled if set in Vision)
            smHeadingDeg  = (smHeadingDeg  == null) ? h : (SMOOTH_A * h + (1 - SMOOTH_A) * smHeadingDeg);
            smRangeMeters = (smRangeMeters == null) ? r : (SMOOTH_A * r + (1 - SMOOTH_A) * smRangeMeters);
        } else {
            // Keep previous smoothed values; they will show as last-known until we see the tag again.
            // (Alternatively, set to null to blank when not visible)
        }

        if (aimEnabled && goalDet != null) {
            // Override only twist — driver retains full translation control.
            // If you wish to aim using smoothed bearing instead, swap to smHeadingDeg below.
            twist = aim.turnPower(goalDet);
        }

        // ==============================================================
        // DRIVEBASE EXECUTION
        // ==============================================================
        drive.drive(driveY, strafeX, twist);

        // ==============================================================
        // BUTTON EDGE DETECTION (INTAKE, FEED, MODE TOGGLES)
        // ==============================================================
        boolean y  = gamepad1.y;
        boolean lb = gamepad1.left_bumper;
        boolean a  = gamepad1.a;

        // Toggle manual speed mode
        if (y && !prevY) manualSpeedMode = !manualSpeedMode;

        // Toggle intake
        if (lb && !prevLB) intake.toggle();

        // Feed one ball (single cycle)
        if (a && !prevA) feed.feedOnceBlocking();

        prevY  = y;
        prevLB = lb;
        prevA  = a;

        // ==============================================================
        // TELEMETRY OUTPUT
        // (Original lines preserved + added vision telemetry)
        // ==============================================================
        telemetry.addData("Alliance", alliance());
        telemetry.addData("BrakeCap", "%.2f", cap);
        telemetry.addData("Intake", intake.isOn() ? "On" : "Off");
        telemetry.addData("ManualSpeed", manualSpeedMode);
        telemetry.addData("RT", "%.2f", gamepad1.right_trigger);
        telemetry.addData("RPM Target", "%.0f", launcher.targetRpm);
        telemetry.addData("RPM Actual", "%.0f", launcher.getCurrentRpm());

        // ---- VISION TELEMETRY (ALWAYS SHOWN) ----
        telemetry.addData("Aim Enabled", aimEnabled);
        telemetry.addData("Target Tag ID", targetId);

        // Show all seen tag IDs (always shown)
        java.util.List<Integer> seenIds = new java.util.ArrayList<>();
        for (AprilTagDetection d : vision.getDetectionsCompat()) seenIds.add(d.id);
        telemetry.addData("Seen Tags (count)", seenIds.size());
        telemetry.addData("Seen Tag IDs", seenIds.isEmpty() ? "[]" : seenIds.toString());


        // Prefer smoothed values if present
        double headingDeg = (smHeadingDeg == null) ? Double.NaN : smHeadingDeg;
        double distM = vision.getScaledRange(goalDet);
        double distIn = Double.isNaN(distM) ? Double.NaN : distM * 39.3701;  // meters → inches

        telemetry.addData("Goal Tag Visible", goalDet != null);
        telemetry.addData("Goal Heading (deg)", Double.isNaN(headingDeg) ? "---" : String.format("%.1f", headingDeg));
        // --- Tag distance telemetry (converted to inches) ---
        telemetry.addData("Tag Distance (in)", Double.isNaN(distIn) ? "---" : String.format("%.1f", distIn));


        telemetry.update();
    }

    @Override
    public void stop() {
        // ---- Cleanup Vision Resources ----
        if (vision != null) vision.stop();
    }
}
