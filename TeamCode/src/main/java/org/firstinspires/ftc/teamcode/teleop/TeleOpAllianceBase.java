package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.drive.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Feed;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;

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
 * - X:                FIRE one ball (runs Feed motor single shot)   [edge]
 * - Left Bumper:      Toggle Intake on/off                          [edge]
 * - Triangle:         Toggle ManualSpeed mode on/off                [edge]
 * - Right Bumper:     (optional placeholder) Aim-assist enable while held
 *
 * TUNABLES:
 * - slowestSpeed:      Cap when fully braking with Left trigger (e.g., 0.25)
 * - rpmBottom/rpmTop:  Manual RPM range mapped from Right trigger
 *
 * IMPORTANT METHODS:
 * - alliance(): implemented by subclasses (TeleOp_Red/TeleOp_Blue)
 * - init():     creates subsystems
 * - loop():     reads gamepad, applies drive/launcher/feed/intake logic, updates telemetry
 *
 * NOTES:
 * - Intake toggle is bound to Left Bumper because Left Trigger is used for braking.
 *   If you prefer L2 for intake, we can remap and change brake behavior accordingly.
 * - Aim-assist is left as a placeholder so it won't fight with RT's RPM mapping.
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
    private double slowestSpeed = 0.25;     // full brake cap
    private double rpmBottom    = 0;     // manual RPM range bottom
    private double rpmTop       = 6000;     // manual RPM range top

    // ---- State ----
    private boolean manualSpeedMode = true; // Triangle toggles this

    // Button edge tracking (so we toggle once per press)
    private boolean prevY  = false; // Triangle
    private boolean prevLB = false; // Left Bumper (intake toggle)
    private boolean prevA  = false; // X/A (fire)

    @Override
    public void init() {
        // Use TeleOp-friendly Drivebase constructor (no shim, no blocking)
        drive    = new Drivebase(hardwareMap, telemetry);
        launcher = new Launcher(hardwareMap);
        feed     = new Feed(hardwareMap);
        intake   = new Intake(hardwareMap);

        telemetry.addData("TeleOp", "Alliance: %s", alliance());
        telemetry.update();
    }

    @Override
    public void loop() {
        // ---------------- Drivetrain with brake scaling ----------------
        double brake = gamepad1.left_trigger; // 0..1
        // Cap scales from 1.0 (no brake) down to slowestSpeed (full brake)
        double cap = 1.0 - brake * (1.0 - slowestSpeed);

        double driveY  = cap * gamepad1.left_stick_y;     // +forward
        double strafeX = cap * -gamepad1.left_stick_x;    // +right (matches your original)
        double twist   = cap * -gamepad1.right_stick_x;   // +CCW  (matches your original)

        // ---- Optional aim-assist placeholder (Right Bumper while held) ----
        // If you later wire AprilTag angle here, inject a small twist correction when RB is held.
        // boolean aimHeld = gamepad1.right_bumper;
        // if (aimHeld && Vision.hasTarget()) {
        //     double err = Vision.getAngleToTarget(); // deg
        //     double kAim = 0.02;                     // tune
        //     twist = Math.max(-cap, Math.min(cap, kAim * err));
        // }

        drive.drive(driveY, strafeX, twist);

        // ---------------- Button edge detection ----------------
        boolean y  = gamepad1.y;
        boolean lb = gamepad1.left_bumper;
        boolean a  = gamepad1.a;

        // Triangle: toggle manual/auto speed mode (edge)
        if (y && !prevY) manualSpeedMode = !manualSpeedMode;

        // Left Bumper: toggle intake (edge)
        if (lb && !prevLB) intake.toggle();

        // X (A): fire one ball (edge)
        if (a && !prevA) {
            // Add interlock when you wire real RPM sensing:
            // if (launcher.isAtSpeed(100)) { feed.feedOnceBlocking(); }
            feed.feedOnceBlocking();
        }

        prevY  = y;
        prevLB = lb;
        prevA  = a;

        // ---------------- Launcher RPM control ----------------
        if (manualSpeedMode) {
            double rt = gamepad1.right_trigger; // 0..1
            if (rt > 0.05) { // deadzone
                double rpm = rpmBottom + rt * (rpmTop - rpmBottom);
                launcher.setTargetRpm(rpm);
            } else {
                launcher.setTargetRpm(0); // no trigger -> stop flywheels
            }
        } else {
            // Auto RPM mode not implemented yet; keep off for now
            launcher.setTargetRpm(0);
        }

        // ---------------- Telemetry ----------------
        telemetry.addData("Alliance", alliance());
        telemetry.addData("BrakeCap", "%.2f", cap);
        telemetry.addData("Intake", intake.isOn() ? "On" : "Off");
        telemetry.addData("ManualSpeed", manualSpeedMode);
        telemetry.addData("RT", "%.2f", gamepad1.right_trigger);
        telemetry.addData("RPM Target", "%.0f", launcher.targetRpm);
        telemetry.addData("RPM Actual", "%.0f", launcher.getCurrentRpm()); // placeholder until closed-loop RPM wired
        telemetry.update();
    }
}
