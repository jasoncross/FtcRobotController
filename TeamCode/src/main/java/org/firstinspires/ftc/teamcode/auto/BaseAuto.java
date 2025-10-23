package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.drive.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Feed;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import org.firstinspires.ftc.teamcode.utils.ObeliskSignal;
import org.firstinspires.ftc.teamcode.vision.VisionAprilTag;
import org.firstinspires.ftc.teamcode.vision.TagAimController;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import org.firstinspires.ftc.teamcode.assist.AutoAimSpeed;
import org.firstinspires.ftc.teamcode.control.LauncherAutoSpeedController;

import org.firstinspires.ftc.teamcode.config.AutoRpmConfig;
import org.firstinspires.ftc.teamcode.config.SharedRobotTuning;

/*
 * ============================================================================
 * FILE: BaseAuto.java
 * PURPOSE:
 *   Abstract base for DECODE Autonomous. Preserves obelisk-latch + HUD,
 *   wires shared AutoAim/AutoSpeed helper, and provides convenience helpers.
 *
 * TUNING — READ THIS FIRST:
 *   • Distance→RPM curve & smoothing: AutoRpmConfig.java
 *   • Shared timing & limits (shot spacing, RPM tolerance, twist/drive caps,
 *     intake assist, initial RPM seed): SharedRobotTuning.java
 *
 * NOTE (WHERE THESE USED TO LIVE):
 *   • Previously, SHOT_BETWEEN_MS / RPM_TOLERANCE / TURN_TWIST_CAP /
 *     DRIVE_MAX_POWER were tuned here; they now come from SharedRobotTuning.
 * ============================================================================
 */
public abstract class BaseAuto extends LinearOpMode {

    // ---- Subsystems ----
    protected Drivebase drive;
    protected VisionAprilTag vision; // optional; may be null if camera absent
    protected Launcher  launcher;
    protected Feed      feed;
    protected Intake    intake;

    // ---- Controllers / helper ----
    protected TagAimController            aimCtrl;
    protected LauncherAutoSpeedController autoCtrl;
    protected AutoAimSpeed                autoAssist;

    // ---- HUD ----
    private volatile boolean hudRunning = false;
    private Thread hudThread = null;

    // ---------------- Template methods ----------------
    protected abstract Alliance alliance();
    protected abstract void runSequence() throws InterruptedException;
    protected void onPreStartLoop() { /* optional */ }

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new Drivebase(this);

        // Vision (best-effort)
        try {
            vision = new VisionAprilTag();
            vision.init(hardwareMap, "Webcam 1");
        } catch (Exception ex) {
            vision = null;
        }

        // Mechs  (match TeleOp constructors: HardwareMap only)
        launcher = new Launcher(hardwareMap);
        feed     = new Feed(hardwareMap);
        intake   = new Intake(hardwareMap);

        // Controllers + helper  (match TeleOp: no-arg TagAimController; no-arg AutoSpeedController)
        aimCtrl  = new TagAimController();
        autoCtrl = new LauncherAutoSpeedController();
        AutoRpmConfig.apply(autoCtrl); // centralized tunables
        autoAssist = new AutoAimSpeed(vision, aimCtrl, autoCtrl, launcher);
        autoAssist.maxTwist = SharedRobotTuning.TURN_TWIST_CAP;
        autoAssist.rpmTolerance = SharedRobotTuning.RPM_TOLERANCE;
        autoAssist.initialAutoDefaultSpeed = SharedRobotTuning.INITIAL_AUTO_DEFAULT_SPEED;

        // Reset obelisk and observe in prestart
        ObeliskSignal.clear();

        while (!isStarted() && !isStopRequested()) {
            if (vision != null) vision.observeObelisk();
            telemetry.addData("Obelisk", ObeliskSignal.getDisplay());
            telemetry.addData("Auto", "Alliance: %s", alliance());
            onPreStartLoop();
            telemetry.update();
            sleep(20);
        }

        if (isStopRequested()) {
            stopVisionIfAny();
            return;
        }

        if (vision != null) vision.setObeliskAutoLatchEnabled(true);
        startHud();

        try {
            runSequence();
        } finally {
            stopHud();
            stopAll();
            stopVisionIfAny();
            telemetry.addLine("Auto complete – DS will queue TeleOp.");
            telemetry.update();
            sleep(750);
        }
    }

    // ==================== Convenience helpers ====================

    protected AprilTagDetection goalDet() {
        if (vision == null) return null;
        int id = (alliance() == Alliance.BLUE) ? VisionAprilTag.TAG_BLUE_GOAL : VisionAprilTag.TAG_RED_GOAL;
        return vision.getDetectionFor(id);
    }

    protected void turnToGoalTag(double timeoutMs) {
        long end = System.currentTimeMillis() + (long)timeoutMs;
        autoAssist.enable();
        while (opModeIsActive() && System.currentTimeMillis() < end) {
            AprilTagDetection det = goalDet();
            double twist = autoAssist.update(det, drive.heading());
            drive.drive(0, 0, twist);
            telemetry.addData("Aim", "twist=%.2f rpm=%.0f", twist, launcher.targetRpm);
            telemetry.update();
        }
        drive.stopAll();
    }

    protected void aimSpinUntilReady(long timeoutMs) {
        long end = System.currentTimeMillis() + timeoutMs;
        autoAssist.enable();
        while (opModeIsActive() && System.currentTimeMillis() < end) {
            AprilTagDetection det = goalDet();
            double twist = autoAssist.update(det, drive.heading());
            drive.drive(0, 0, twist);
            boolean atSpd = autoAssist.atSpeed(SharedRobotTuning.RPM_TOLERANCE);
            telemetry.addData("AutoSpeed", "tgt=%.0f at=%s", launcher.targetRpm, atSpd);
            telemetry.update();
            if (atSpd) break;
        }
        drive.stopAll();
    }

    protected void fireOnceWithIntakeAssist() throws InterruptedException {
        boolean wasOn = intake.isOn();
        if (!wasOn) intake.set(true);
        feed.feedOnceBlocking();
        if (!wasOn) {
            sleep(SharedRobotTuning.INTAKE_ASSIST_MS);
            intake.set(false);
        }
    }

    protected void fireN(int count) throws InterruptedException {
        for (int i = 0; i < count && opModeIsActive(); i++) {
            fireOnceWithIntakeAssist();
            if (i < count - 1) sleep(SharedRobotTuning.SHOT_BETWEEN_MS);
        }
    }

    protected void turnBackTo(double targetHeadingDeg) {
        double cur = drive.heading();
        double diff = shortestDiff(targetHeadingDeg, cur);
        drive.turn(diff, clamp(SharedRobotTuning.TURN_TWIST_CAP, 0.2, 1.0));
    }

    protected void driveForwardInches(double inches) {
        drive.move(inches, /*degrees=*/0, clamp(SharedRobotTuning.DRIVE_MAX_POWER, 0.1, 1.0));
    }

    protected void stopAll() {
        try { drive.stopAll(); } catch (Exception ignored) {}
        try { launcher.stop(); }  catch (Exception ignored) {}
        try { feed.stop(); }      catch (Exception ignored) {}
        try { intake.stop(); }    catch (Exception ignored) {}
        try { autoAssist.disable(); } catch (Exception ignored) {}
    }

    // ==================== Vision / HUD + math ====================

    protected final void stopVisionIfAny() {
        try {
            if (vision != null) {
                vision.setObeliskAutoLatchEnabled(false);
                vision.stop();
            }
        } catch (Exception ignored) { }
    }

    protected final void startHud() {
        if (hudRunning) return;
        hudRunning = true;
        hudThread = new Thread(() -> {
            try {
                while (hudRunning && !isStopRequested()) {
                    try {
                        telemetry.addData("Obelisk", ObeliskSignal.getDisplay());
                        telemetry.addData("Auto", "Alliance: %s", alliance());
                        telemetry.update();
                    } catch (Exception ignored) { }
                    try { Thread.sleep(200); } catch (InterruptedException ie) { break; }
                }
            } finally { hudRunning = false; }
        }, "AutoHUD");
        hudThread.setDaemon(true);
        hudThread.start();
    }

    protected final void stopHud() {
        hudRunning = false;
        if (hudThread != null) {
            try { hudThread.join(150); } catch (InterruptedException ignored) {}
            hudThread = null;
        }
    }

    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
    private static double shortestDiff(double target, double current) {
        double d = ((target - current) % 360 + 360) % 360;
        if (d > 180) d -= 360;
        return d;
    }
}
