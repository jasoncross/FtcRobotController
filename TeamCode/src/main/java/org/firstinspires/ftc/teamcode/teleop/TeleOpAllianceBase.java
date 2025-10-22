package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.drive.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Feed;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;

// === NEW: vision imports ===
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.vision.VisionAprilTag;
import org.firstinspires.ftc.teamcode.vision.TagAimController;

/*
 * FILE: TeleOpAllianceBase.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleop/
 *
 * PURPOSE:
 * - Shared TeleOp base used by TeleOp_Red and TeleOp_Blue.
 * - Wires the drivetrain (Drivebase), Launcher, Feed, and Intake subsystems.
 * - Implements the control mapping with button edge-detection so toggles
 *   flip exactly once per press.
 *
 * CONTROLS (PS layout; adjust if using Xbox/Logitech):
 * - Left stick (Y/X): forward/back + strafe
 * - Right stick X:    twist/rotation
 * - Left trigger:     BRAKE (scales power down toward slowestSpeed)
 * - Right trigger:    Manual launch RPM (only when manualSpeedMode == true)
 * - X (A):            FIRE one ball (runs Feed motor single shot)         [edge]
 * - Left Bumper:      Toggle Intake on/off                                [edge]
 * - Triangle:         Toggle ManualSpeed mode on/off                      [edge]
 * - Right Bumper:     TOGGLE Aim-Assist (keep robot aimed at alliance tag) [edge]
 *
 * TUNABLES:
 * - slowestSpeed:   Cap when fully braking with Left trigger (e.g. 0.25)
 * - rpmBottom/Top:  Manual RPM range mapped from Right trigger
 *
 * NEW (VISION):
 * - Aim-Assist uses AprilTag bearing to override twist only; forward/back + strafe remain
 *   under driver control. When enabled but no tag is visible, we fall back to manual twist.
 *
 * IMPORTANT METHODS:
 * - alliance(): implemented by subclasses (TeleOp_Red/TeleOp_Blue)
 * - init():     creates subsystems (+ vision)
 * - loop():     applies drive/launcher/feed/intake logic (+ aim), updates telemetry
 * - stop():     closes vision resources
 */

public abstract class TeleOpAllianceBase extends OpMode {
    // Implemented by TeleOp_Red / TeleOp_Blue
    protected abstract Alliance alliance();

    // Subsystems
    protected Drivebase drive;
    protected Launcher launcher;
    protected Feed feed;
    protected Intake intake;

    // ---- Tunables ----
    private double slowestSpeed = 0.25;  // full brake cap
    private double rpmBottom    = 0;     // manual RPM range bottom
    private double rpmTop       = 6000;  // manual RPM range top

    // ---- State ----
    private boolean manualSpeedMode = true; // Triangle toggles this

    // Button edge tracking (so we toggle once per press)
    private boolean prevY  = false; // Triangle
    private boolean prevLB = false; // Left Bumper (intake toggle)
    private boolean prevA  = false; // X/A (fire)
    private boolean prevRB = false; // Right Bumper (aim toggle)

    // ==== NEW: Vision + Aim state ====
    private VisionAprilTag vision;
    private TagAimController aim = new TagAimController();
    private boolean aimEnabled = false;  // toggled via Right Bumper

    @Override
    public void init() {
        // Use TeleOp-friendly Drivebase constructor (no shim, no blocking)
        drive    = new Drivebase(hardwareMap, telemetry);
        launcher = new Launcher(hardwareMap);
        feed     = new Feed(hardwareMap);
        intake   = new Intake(hardwareMap);

        // === NEW: VisionPortal + AprilTag ===
        vision = new VisionAprilTag();
        vision.init(hardwareMap, "Webcam 1"); // must match Robot Configuration

        telemetry.addData("TeleOp", "Alliance: %s", alliance());
        telemetry.addLine("Aim: Right Bumper toggles ON/OFF");
        telemetry.update();
    }

    @Override
    public void loop() {
        // ---------------- Drivetrain with brake scaling ----------------
        double brake = gamepad1.left_trigger; // 0..1
        // Cap scales from 1.0 (no brake) down to slowestSpeed (full brake)
        double cap = 1.0 - brake * (1.0 - slowestSpeed);

        double driveY  = cap * gamepad1.left_stick_y;   // +forward
        double strafeX = cap * -gamepad1.left_stick_x;  // +right (matches original sign)
        double twist   = cap * -gamepad1.right_stick_x; // +CCW  (matches original sign)

        // ---------------- Aim toggle (Right Bumper) [edge] ----------------
        boolean rb = gamepad1.right_bumper;
        if (rb && !prevRB) {
            aimEnabled = !aimEnabled;
        }
        prevRB = rb;

        // ---------------- Alliance tag selection ----------------
        int targetId = (alliance() == Alliance.BLUE)
                ? VisionAprilTag.TAG_BLUE_GOAL   // 20
                : VisionAprilTag.TAG_RED_GOAL;   // 24

        // ---------------- Aim override (twist only) ----------------
        AprilTagDetection det = null;
        if (aimEnabled) {
            det = vision.getDetectionFor(targetId);
            if (det != null) {
                // Override twist with controller output so we keep facing the tag;
                // translation (driveY/strafeX) remains under driver control.
                twist = aim.turnPower(det);
            }
        }

        // ---------------- Drive ----------------
        drive.drive(driveY, strafeX, twist);

        // ---------------- Button edge detection ----------------
        boolean y  = gamepad1.y;
        boolean lb = gamepad1.left_bumper;
        boolean a  = gamepad1.a;

        // Triangle: toggle manual/auto speed mode (edge)
        if (y && !prevY) manualSpeedMode = !manualSpeedMode;

        // Left Bumper: toggle intake (edge)
        if (lb && !prevLB) intake.toggle();

        // X/A: fire once (edge)
        if (a && !prevA) feed.fireOnce();

        prevY  = y;
        prevLB = lb;
        prevA  = a;

        // ---------------- Telemetry ----------------
        telemetry.addData("Alliance", alliance());
        telemetry.addData("BrakeCap", "%.2f", cap);
        telemetry.addData("Intake", intake.isOn() ? "On" : "Off");
        telemetry.addData("ManualSpeed", manualSpeedMode);
        telemetry.addData("RT", "%.2f", gamepad1.right_trigger);
        telemetry.addData("RPM Target", "%.0f", launcher.targetRpm);
        telemetry.addData("RPM Actual", "%.0f", launcher.getCurrentRpm()); // placeholder until closed-loop RPM wired

        // --- NEW vision/aim telemetry appended without removing any existing lines ---
        telemetry.addData("Aim Enabled", aimEnabled);
        if (aimEnabled) {
            double headingDeg = TagAimController.headingDeg(det);
            double distM      = TagAimController.distanceMeters(det);
            telemetry.addData("Tag Visible", det != null);
            telemetry.addData("Tag Heading (deg)", Double.isNaN(headingDeg) ? "---" : String.format("%.1f", headingDeg));
            telemetry.addData("Tag Distance (m)",  Double.isNaN(distM) ? "---" : String.format("%.2f", distM));
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        // Close vision resources
        if (vision != null) vision.stop();
    }
}
