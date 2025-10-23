package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.drive.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Feed;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.utils.ObeliskSignal;
import org.firstinspires.ftc.teamcode.vision.VisionAprilTag;
import org.firstinspires.ftc.teamcode.vision.TagAimController;
import org.firstinspires.ftc.teamcode.control.LauncherAutoSpeedController;

// If you centralized tunables, these will resolve. If not, defaults below take over.
import org.firstinspires.ftc.teamcode.config.AutoRpmConfig;
import org.firstinspires.ftc.teamcode.config.SharedRobotTuning;

/*
 * ============================================================================
 * FILE: BaseAuto.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/
 *
 * PURPOSE:
 *   Abstract base for all Autonomous modes. Provides:
 *     - Subsystem setup/teardown + StopAll safety
 *     - Obelisk (tags 21/22/23) latch in prestart and during Auto
 *     - Lightweight HUD
 *     - Tag-gated helpers that keep robot stationary while shooting:
 *         • turnToGoalTag(timeoutMs): sweep with a mode-specific initial direction;
 *           center to ≤ lock tolerance deg and STOP.
 *         • aimSpinUntilReady(timeoutMs): AutoSpeed to within ±RPM tolerance; STOP drive.
 *         • fireN(count): re-check lock + RPM before each ball, spacing between shots.
 *         • turnBackTo(startHeadingDeg), driveForwardInches(inches)
 *
 * NOTES:
 *   • Scan direction now comes from each Auto via initialScanCW() so we can prefer
 *     the most likely goal direction based on the real start pose.
 *   • DS init shows a "Start Pose" line describing where the robot starts and faces.
 * ============================================================================
 */
public abstract class BaseAuto extends LinearOpMode {

    // ----------------------- Required by concrete Autos -----------------------
    protected abstract Alliance alliance();
    /** One-line description of where the robot starts and which way it faces. Shown on DS during init. */
    protected abstract String startPoseDescription();
    /** Preferred first scan direction for the goal tag. Return true for CW (right), false for CCW (left). */
    protected abstract boolean initialScanCW();

    /** Implement the autonomous steps here. */
    protected abstract void runSequence() throws InterruptedException;

    /** Optional hook for extra prestart telemetry. */
    protected void onPreStartLoop() { /* no-op */ }

    // ----------------------- Subsystems --------------------------------------
    protected Drivebase drive;
    protected VisionAprilTag vision;
    protected Launcher launcher;
    protected Feed feed;
    protected Intake intake;

    // ----------------------- Controllers -------------------------------------
    protected final TagAimController aim = new TagAimController();
    protected final LauncherAutoSpeedController autoCtrl = new LauncherAutoSpeedController();

    // ----------------------- HUD ---------------------------------------------
    private volatile boolean hudRunning = false;
    private Thread hudThread = null;

    // ----------------------- Defaults if configs are absent -------------------
    private static final double DEFAULT_LOCK_TOL_DEG   = 1.0;
    private static final double DEFAULT_TURN_TWIST_CAP = 0.35;
    private static final double DEFAULT_DRIVE_CAP      = 0.50;
    private static final double DEFAULT_RPM_TOL        = 50.0;
    private static final long   DEFAULT_BETWEEN_MS     = 3000;
    private static final double DEFAULT_INIT_AUTO_RPM  = 2500.0;

    private double lockTolDeg() {
        try { return Math.max(0.2, SharedRobotTuning.LOCK_TOLERANCE_DEG); }
        catch (Throwable t) { return DEFAULT_LOCK_TOL_DEG; }
    }
    private double turnTwistCap() {
        try { return clamp(SharedRobotTuning.TURN_TWIST_CAP, 0.15, 0.9); }
        catch (Throwable t) { return DEFAULT_TURN_TWIST_CAP; }
    }
    private double driveCap() {
        try { return clamp(SharedRobotTuning.DRIVE_MAX_POWER, 0.2, 1.0); }
        catch (Throwable t) { return DEFAULT_DRIVE_CAP; }
    }
    private double rpmTol() {
        try { return Math.max(10, SharedRobotTuning.RPM_TOLERANCE); }
        catch (Throwable t) { return DEFAULT_RPM_TOL; }
    }
    private long betweenShotsMs() {
        try { return Math.max(250, SharedRobotTuning.SHOT_BETWEEN_MS); }
        catch (Throwable t) { return DEFAULT_BETWEEN_MS; }
    }
    private double initialAutoSeedRpm() {
        try { return SharedRobotTuning.INITIAL_AUTO_DEFAULT_SPEED; }
        catch (Throwable t) { return DEFAULT_INIT_AUTO_RPM; }
    }

    // ----------------------- Lifecycle ---------------------------------------
    @Override
    public void runOpMode() throws InterruptedException {
        // Drivebase (Auto constructor enables blocking move/turn)
        drive = new Drivebase(this);

        // Vision (best-effort)
        try {
            vision = new VisionAprilTag();
            vision.init(hardwareMap, "Webcam 1");
        } catch (Exception ex) { vision = null; }

        // Subsystems
        launcher = new Launcher(hardwareMap);
        feed     = new Feed(hardwareMap);
        intake   = new Intake(hardwareMap);
        intake.set(false);

        // Auto RPM curve
        try { AutoRpmConfig.apply(autoCtrl); } catch (Throwable ignored) {}

        // Reset obelisk for this Auto
        ObeliskSignal.clear();

        // --------------------- Pre-Start Loop --------------------------------
        while (!isStarted() && !isStopRequested()) {
            if (vision != null) vision.observeObelisk();

            telemetry.addData("Obelisk", ObeliskSignal.getDisplay());
            telemetry.addData("Auto", "Alliance: %s", alliance());
            telemetry.addData("Start Pose", startPoseDescription()); // <<< NEW, visible on DS
            onPreStartLoop();
            telemetry.update();
            sleep(20);
        }

        if (isStopRequested()) { stopVisionIfAny(); return; }

        // Enable background latch + HUD
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
            sleep(250);
        }
    }

    // =========================================================================
    //  Helpers used by Auto_* sequences
    // =========================================================================

    /** Sweep to find the goal tag, then center to within tolerance. Keeps robot stopped when centered. */
    protected final boolean turnToGoalTag(long timeoutMs) {
        final int goalId = (alliance() == Alliance.BLUE)
                ? VisionAprilTag.TAG_BLUE_GOAL
                : VisionAprilTag.TAG_RED_GOAL;

        final double tol = lockTolDeg();
        final double twistCap = turnTwistCap();

        long start = System.currentTimeMillis();
        long lastDirFlip = start;

        // Prefer a mode-specific initial search direction (CW vs CCW)
        // CW (right) = negative twist in drive.drive(drive, strafe, twist)
        double scanSign = initialScanCW() ? -1.0 : +1.0;

        while (opModeIsActive() && (System.currentTimeMillis() - start) < timeoutMs) {
            AprilTagDetection det = (vision != null) ? vision.getDetectionFor(goalId) : null;

            if (det != null) {
                double err = det.ftcPose.bearing;
                if (Math.abs(err) <= tol) {
                    drive.stopAll();
                    return true;
                }
                double cmd = clamp(aim.turnPower(det), -twistCap, +twistCap);
                drive.drive(0, 0, cmd);
            } else {
                // Slow scan; flip direction every 700ms so we don't drift away forever
                long now = System.currentTimeMillis();
                if (now - lastDirFlip > 700) {
                    scanSign *= -1.0;
                    lastDirFlip = now;
                }
                drive.drive(0, 0, scanSign * 0.25 * twistCap);
            }
            idle();
        }

        drive.stopAll();
        return false;
    }

    /** Hold still and use AutoSpeed until RPM is within ±rpmTol (tag distance if available). */
    protected final boolean aimSpinUntilReady(long timeoutMs) {
        drive.stopAll();

        autoCtrl.setAutoEnabled(true);
        try { AutoRpmConfig.apply(autoCtrl); } catch (Throwable ignored) {}

        final double M_TO_IN = 39.37007874015748;
        final int goalId = (alliance() == Alliance.BLUE)
                ? VisionAprilTag.TAG_BLUE_GOAL
                : VisionAprilTag.TAG_RED_GOAL;

        boolean hadFix = false;
        long start = System.currentTimeMillis();

        while (opModeIsActive() && (System.currentTimeMillis() - start) < timeoutMs) {
            AprilTagDetection det = (vision != null) ? vision.getDetectionFor(goalId) : null;
            Double distIn = null;
            if (det != null) {
                double rM_sc = vision.getScaledRange(det);
                if (!Double.isNaN(rM_sc) && Double.isFinite(rM_sc)) {
                    distIn = rM_sc * M_TO_IN;
                    hadFix = true;
                }
            }

            if (distIn == null && !hadFix) {
                launcher.setTargetRpm(initialAutoSeedRpm());
            } else {
                double rpm = autoCtrl.updateWithVision(distIn);
                launcher.setTargetRpm(rpm);
            }

            double at = launcher.getCurrentRpm();
            if (Math.abs(at - launcher.targetRpm) <= rpmTol()) return true;
            idle();
        }
        return false;
    }

    /** Fire N balls; before each ball re-check lock (≤tol) and at-speed (±rpmTol). Robot stays still. */
    protected final void fireN(int count) throws InterruptedException {
        final double tol = lockTolDeg();
        final double cap = turnTwistCap();
        final long guardMs = 700;
        final int goalId = (alliance() == Alliance.BLUE)
                ? VisionAprilTag.TAG_BLUE_GOAL
                : VisionAprilTag.TAG_RED_GOAL;

        for (int i = 0; i < count && opModeIsActive(); i++) {
            // Re-acquire / micro-center
            long guardStart = System.currentTimeMillis();
            while (opModeIsActive()) {
                AprilTagDetection det = (vision != null) ? vision.getDetectionFor(goalId) : null;
                if (det != null) {
                    double err = det.ftcPose.bearing;
                    if (Math.abs(err) <= tol) break;
                    double cmd = clamp(aim.turnPower(det), -cap, +cap);
                    drive.drive(0, 0, cmd * 0.6);
                } else {
                    drive.stopAll();
                }
                if (System.currentTimeMillis() - guardStart > guardMs) break;
                idle();
            }

            // Require at-speed
            while (opModeIsActive()) {
                if (Math.abs(launcher.getCurrentRpm() - launcher.targetRpm) <= rpmTol()) break;
                idle();
            }

            // Feed once with intake assist like TeleOp
            boolean wasOn = intake.isOn();
            if (!wasOn) intake.set(true);
            feed.feedOnceBlocking();
            if (!wasOn) {
                int assist;
                try { assist = SharedRobotTuning.INTAKE_ASSIST_MS; }
                catch (Throwable t) { assist = 250; }
                sleep(assist);
                intake.set(false);
            }

            sleep((int)betweenShotsMs());
            drive.stopAll();
        }
    }

    /** Turn back to a heading (deg) using Drivebase turn(). */
    protected final void turnBackTo(double startHeadingDeg) {
        double cur = drive.heading();
        double delta = shortestDiff(startHeadingDeg, cur);
        drive.turn(delta, clamp(turnTwistCap() + 0.05, 0.2, 0.8));
    }

    /** Drive forward inches at a conservative cap (encoder RUN_TO_POSITION). */
    protected final void driveForwardInches(double inches) {
        drive.move(inches, 0.0, driveCap());
    }

    // ----------------------- Safety/utility ----------------------------------
    protected final void stopAll() {
        try { drive.stop(); } catch (Throwable ignored) {}
        try { launcher.stop(); } catch (Throwable ignored) {}
        try { feed.stop(); } catch (Throwable ignored) {}
        try { intake.stop(); } catch (Throwable ignored) {}
    }

    protected final void stopVisionIfAny() {
        try {
            if (vision != null) {
                vision.setObeliskAutoLatchEnabled(false);
                vision.stop();
            }
        } catch (Exception ignored) {}
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
                        telemetry.addData("Start Pose", startPoseDescription()); // keep showing start info
                        telemetry.update();
                    } catch (Exception ignored) {}
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

    // ----------------------- math --------------------------------------------
    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
    private static double shortestDiff(double target, double current) {
        double d = normDeg(target - current);
        if (d > 180) d -= 360;
        if (d < -180) d += 360;
        return d;
    }
    private static double normDeg(double a) {
        double r = a % 360;
        if (r < 0) r += 360;
        return r;
    }
}
