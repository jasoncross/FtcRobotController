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
 * - Shared TeleOp base used by TeleOp_Red and TeleOp_Blue.
 * - Wires the drivetrain (Drivebase), Launcher, Feed, and Intake subsystems.
 * - Implements button edge-detection for toggles and manual controls.
 * - Integrates AprilTag-based Aim-Assist (Right Bumper toggle).
 *
 * CONTROLS (PS layout; adjust if using Xbox/Logitech):
 * - Left stick (Y/X): Forward/back + strafe.
 * - Right stick X:    Twist/rotation.
 * - Left trigger:     Brake (scales power down toward slowestSpeed).
 * - Right trigger:    Manual launch RPM (only when manualSpeedMode == true).
 * - X (A):            Feed one ball (runs Feed motor single shot).          [edge]
 * - Left Bumper:      Toggle Intake ON/OFF.                                [edge]
 * - Triangle (Y):     Toggle ManualSpeed mode ON/OFF.                      [edge]
 * - Right Bumper:     Toggle Aim-Assist (keeps robot aimed at alliance tag). [edge]
 *
 * TUNABLES:
 * - slowestSpeed:   Cap when fully braking with Left Trigger (e.g., 0.25).
 * - rpmBottom/Top:  Manual RPM range mapped from Right Trigger.
 *
 * NEW (VISION):
 * - Aim-Assist overrides *only twist* to keep the robot facing the AprilTag.
 * - Forward/back + strafe remain fully driver-controlled.
 * - Always displays tag telemetry (distance, heading, visibility).
 * - Adds simple smoothing for heading & distance to reduce jitter.
 * - Telemetry shows distance in INCHES (converted from meters).
 *
 * METHODS:
 * - alliance(): Implemented by subclasses (TeleOp_Red / TeleOp_Blue).
 * - init():     Initializes subsystems and VisionPortal.
 * - loop():     Main control logic and telemetry updates.
 * - stop():     Shuts down vision resources cleanly.
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

        // Show all seen tag IDs for quick debugging
        List<Integer> seenIds = new ArrayList<>();
        for (AprilTagDetection d : vision.getDetections()) seenIds.add(d.id);
        telemetry.addData("Seen Tags (count)", seenIds.size());
        telemetry.addData("Seen Tag IDs", seenIds.isEmpty() ? "[]" : seenIds.toString());

        // Prefer smoothed values if present
        double headingDeg = (smHeadingDeg == null) ? Double.NaN : smHeadingDeg;
        double distM      = (smRangeMeters == null) ? Double.NaN : smRangeMeters;
        double distIn     = Double.isNaN(distM) ? Double.NaN : (distM * M_TO_IN);

        telemetry.addData("Goal Tag Visible", goalDet != null);
        telemetry.addData("Goal Heading (deg)", Double.isNaN(headingDeg) ? "---" : String.format("%.1f", headingDeg));
        telemetry.addData("Goal Distance (in)", Double.isNaN(distIn)     ? "---" : String.format("%.1f", distIn));

        telemetry.update();
    }

    @Override
    public void stop() {
        // ---- Cleanup Vision Resources ----
        if (vision != null) vision.stop();
    }
}
