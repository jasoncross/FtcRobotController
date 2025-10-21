package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.drive.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Feed;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.util.OpModeShim;

/*
 * FILE: TeleOpAllianceBase.java
 * LOCATION: teamcode/.../teleop/
 *
 * PURPOSE:
 * - Shared TeleOp base for Red and Blue that wires drive, launcher, feed, intake,
 *   and implements the control mapping (sticks, triggers, buttons).
 *
 * TUNABLES:
 * - slowestSpeed: cap applied when Left Trigger is fully pressed (brake mode).
 * - rpmBottom / rpmTop: manual RPM range when Triangle (manual mode) is ON.
 * - aim assist: add gains when you wire Vision (R2 held).
 *
 * IMPORTANT FUNCTIONS:
 * - alliance(): implemented by subclasses to return Alliance.RED/BLUE.
 * - init(): creates Drivebase via OpModeShim and initializes subsystems.
 * - loop(): reads gamepad, runs drive, feed/intake/launcher logic, updates telemetry.
 */
public abstract class TeleOpAllianceBase extends OpMode {
    protected abstract Alliance alliance();

    protected Drivebase drive;
    protected Launcher launcher;
    protected Feed feed;
    protected Intake intake;

    private double slowestSpeed = 0.25;
    private boolean manualSpeedMode = true;
    private double rpmBottom = 2000, rpmTop = 4500;

    @Override
    public void init() {
        drive    = new Drivebase(new OpModeShim(this));
        launcher = new Launcher(hardwareMap);
        feed     = new Feed(hardwareMap);
        intake   = new Intake(hardwareMap);

        telemetry.addData("TeleOp", "Alliance: %s", alliance());
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- Drivetrain with brake scaling (Left Trigger) ---
        double brake = gamepad1.left_trigger; // 0..1
        double cap = 1.0 - brake * (1.0 - slowestSpeed);

        double driveY  = cap * gamepad1.left_stick_y;     // +forward
        double strafeX = cap * -gamepad1.left_stick_x;    // +right
        double twist   = cap * -gamepad1.right_stick_x;   // +CCW

        // --- Aim Assist (R2) placeholder: inject twist from vision angle later ---
        if (gamepad1.right_bumper) { // (use RB as a temporary aim button if RT used for RPM)
            // TODO: twist = kAim * angleError; clamp; skip if no target
        }

        drive.drive(driveY, strafeX, twist);

        // Intake toggle (L2)
        if (gamepad1.left_bumper) { intake.toggle(); }

        // Manual speed mode toggle (Triangle/Y)
        if (gamepad1.y) { manualSpeedMode = !manualSpeedMode; }

        // Manual RPM from Right Trigger when in manual mode
        if (manualSpeedMode) {
            double t = gamepad1.right_trigger; // 0..1
            launcher.setTargetRpm(rpmBottom + t * (rpmTop - rpmBottom));
        }

        // Fire one ball (X/A)
        if (gamepad1.x) {
            // Optional interlock: if (launcher.isAtSpeed(100)) { feed.feedOnceBlocking(); }
            feed.feedOnceBlocking();
        }

        // --- Telemetry ---
        telemetry.addData("Alliance", alliance());
        telemetry.addData("Intake", intake.isOn() ? "On" : "Off");
        telemetry.addData("ManualSpeed", manualSpeedMode);
        telemetry.addData("RPM Target", "%.0f", launcher.targetRpm);
        telemetry.addData("ThrottleCap", "%.2f", cap);
        telemetry.update();
    }
}
