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
import org.firstinspires.ftc.teamcode.config.AutoRpmConfig;
import org.firstinspires.ftc.teamcode.config.SharedRobotTuning;

/*
 * Base autonomous with strict "no tag, no fire".
 * Shows Start Pose on DS; uses initialScanCW() per mode for the first scan direction.
 */
public abstract class BaseAuto extends LinearOpMode {

    protected abstract Alliance alliance();
    protected abstract String startPoseDescription();
    protected abstract boolean initialScanCW();
    protected abstract void runSequence() throws InterruptedException;

    protected void onPreStartLoop() {}

    protected Drivebase drive;
    protected VisionAprilTag vision;
    protected Launcher launcher;
    protected Feed feed;
    protected Intake intake;

    protected final TagAimController aim = new TagAimController();
    protected final LauncherAutoSpeedController autoCtrl = new LauncherAutoSpeedController();

    // Local defaults if config is missing
    private static final double DEF_LOCK_TOL_DEG   = 1.0;
    private static final double DEF_TURN_CAP       = 0.35;
    private static final double DEF_DRIVE_CAP      = 0.50;
    private static final double DEF_RPM_TOL        = 50.0;
    private static final long   DEF_BETWEEN_MS     = 3000;
    private static final double DEF_INIT_RPM       = 2500.0;

    private double lockTolDeg()   { try { return SharedRobotTuning.LOCK_TOLERANCE_DEG; } catch (Throwable t){ return DEF_LOCK_TOL_DEG; } }
    private double turnTwistCap() { try { return SharedRobotTuning.TURN_TWIST_CAP;     } catch (Throwable t){ return DEF_TURN_CAP;     } }
    private double driveCap()     { try { return SharedRobotTuning.DRIVE_MAX_POWER;    } catch (Throwable t){ return DEF_DRIVE_CAP;    } }
    private double rpmTol()       { try { return SharedRobotTuning.RPM_TOLERANCE;      } catch (Throwable t){ return DEF_RPM_TOL;      } }
    private long betweenShotsMs() { try { return SharedRobotTuning.SHOT_BETWEEN_MS;    } catch (Throwable t){ return DEF_BETWEEN_MS;   } }
    private double initAutoRpm()  { try { return SharedRobotTuning.INITIAL_AUTO_DEFAULT_SPEED; } catch (Throwable t){ return DEF_INIT_RPM; } }

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new Drivebase(this);
        try { vision = new VisionAprilTag(); vision.init(hardwareMap, "Webcam 1"); } catch (Exception ex) { vision = null; }
        launcher = new Launcher(hardwareMap);
        feed     = new Feed(hardwareMap);
        intake   = new Intake(hardwareMap);
        intake.set(false);

        try { AutoRpmConfig.apply(autoCtrl); } catch (Throwable ignored) {}
        ObeliskSignal.clear();

        while (!isStarted() && !isStopRequested()) {
            if (vision != null) vision.observeObelisk();
            telemetry.addData("Obelisk", ObeliskSignal.getDisplay());
            telemetry.addData("Auto", "Alliance: %s", alliance());
            telemetry.addData("Start Pose", startPoseDescription());
            onPreStartLoop();
            telemetry.update();
            sleep(20);
        }
        if (isStopRequested()) { stopVisionIfAny(); return; }
        if (vision != null) vision.setObeliskAutoLatchEnabled(true);

        try { runSequence(); }
        finally {
            stopAll();
            stopVisionIfAny();
            telemetry.addLine("Auto complete – DS will queue TeleOp."); telemetry.update();
            sleep(250);
        }
    }

    // ---------- Tag search/center ----------
    protected final boolean turnToGoalTag(long timeoutMs) {
        final int goalId = (alliance() == Alliance.BLUE) ? VisionAprilTag.TAG_BLUE_GOAL : VisionAprilTag.TAG_RED_GOAL;
        final double tol = lockTolDeg();
        final double cap = turnTwistCap();

        long start = System.currentTimeMillis();
        long lastFlip = start;
        double scanSign = initialScanCW() ? -1.0 : +1.0; // CW=negative twist

        while (opModeIsActive() && (System.currentTimeMillis() - start) < timeoutMs) {
            AprilTagDetection det = (vision != null) ? vision.getDetectionFor(goalId) : null;
            if (det != null) {
                double err = det.ftcPose.bearing;
                if (Math.abs(err) <= tol) { drive.stopAll(); return true; }
                double cmd = clamp(aim.turnPower(det), -cap, +cap);
                drive.drive(0, 0, cmd);
            } else {
                long now = System.currentTimeMillis();
                if (now - lastFlip > 700) { scanSign *= -1.0; lastFlip = now; }
                drive.drive(0, 0, scanSign * 0.25 * cap);
            }
            idle();
        }
        drive.stopAll();
        return false;
    }

    // ---------- Spin to at-speed ----------
    protected final boolean aimSpinUntilReady(long timeoutMs) {
        drive.stopAll();
        autoCtrl.setAutoEnabled(true);
        try { AutoRpmConfig.apply(autoCtrl); } catch (Throwable ignored) {}

        final int goalId = (alliance() == Alliance.BLUE) ? VisionAprilTag.TAG_BLUE_GOAL : VisionAprilTag.TAG_RED_GOAL;
        final double M_TO_IN = 39.37007874015748;
        boolean hadFix = false;
        long start = System.currentTimeMillis();

        while (opModeIsActive() && (System.currentTimeMillis() - start) < timeoutMs) {
            AprilTagDetection det = (vision != null) ? vision.getDetectionFor(goalId) : null;
            Double in = null;
            if (det != null) {
                double rM = vision.getScaledRange(det);
                if (!Double.isNaN(rM) && Double.isFinite(rM)) { in = rM * M_TO_IN; hadFix = true; }
            }

            if (in == null && !hadFix) {
                launcher.setTargetRpm(initAutoRpm());
            } else {
                launcher.setTargetRpm(autoCtrl.updateWithVision(in));
            }

            if (Math.abs(launcher.getCurrentRpm() - launcher.targetRpm) <= rpmTol()) return true;
            idle();
        }
        return false;
    }

    // ---------- Strictly gated shooting ----------
    protected final void fireN(int count) throws InterruptedException {
        for (int i = 0; i < count && opModeIsActive(); i++) {
            // REQUIRE a valid lock before each shot (or skip this shot)
            if (!requireLockOrTimeOut(1200)) {
                telemetry.addLine("⚠️ No tag lock — skipping shot " + (i+1));
                telemetry.update();
                continue; // do not free-fire
            }

            // REQUIRE at-speed
            while (opModeIsActive()) {
                if (Math.abs(launcher.getCurrentRpm() - launcher.targetRpm) <= rpmTol()) break;
                idle();
            }

            // Feed once with intake assist
            boolean wasOn = intake.isOn();
            if (!wasOn) intake.set(true);
            feed.feedOnceBlocking();
            if (!wasOn) {
                int assist; try { assist = SharedRobotTuning.INTAKE_ASSIST_MS; } catch (Throwable t) { assist = 250; }
                sleep(assist); intake.set(false);
            }

            sleep((int)betweenShotsMs());
            drive.stopAll();
        }
    }

    /** Wait up to guardMs to achieve a tag lock (|bearing| ≤ tol). Returns true if locked. */
    private boolean requireLockOrTimeOut(long guardMs) {
        final int goalId = (alliance() == Alliance.BLUE) ? VisionAprilTag.TAG_BLUE_GOAL : VisionAprilTag.TAG_RED_GOAL;
        final double tol = lockTolDeg();
        final double cap = turnTwistCap();
        long start = System.currentTimeMillis();

        while (opModeIsActive() && (System.currentTimeMillis() - start) < guardMs) {
            AprilTagDetection det = (vision != null) ? vision.getDetectionFor(goalId) : null;
            if (det != null) {
                double err = det.ftcPose.bearing;
                if (Math.abs(err) <= tol) { drive.stopAll(); return true; }
                double cmd = clamp(aim.turnPower(det), -cap, +cap);
                drive.drive(0, 0, cmd * 0.6);
            } else {
                drive.stopAll();
            }
            idle();
        }
        drive.stopAll();
        return false;
    }

    protected final void turnBackTo(double startHeadingDeg) {
        double cur = drive.heading();
        double delta = shortestDiff(startHeadingDeg, cur);
        drive.turn(delta, clamp(turnTwistCap() + 0.05, 0.2, 0.8));
    }

    protected final void driveForwardInches(double inches) {
        drive.move(inches, 0.0, driveCap());
    }

    protected final void stopAll() {
        try { drive.stop(); } catch (Throwable ignored) {}
        try { launcher.stop(); } catch (Throwable ignored) {}
        try { feed.stop(); } catch (Throwable ignored) {}
        try { intake.stop(); } catch (Throwable ignored) {}
    }
    protected final void stopVisionIfAny() {
        try { if (vision != null) { vision.setObeliskAutoLatchEnabled(false); vision.stop(); } } catch (Exception ignored) {}
    }

    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
    private static double shortestDiff(double target, double current) {
        double d = normDeg(target - current); if (d > 180) d -= 360; if (d < -180) d += 360; return d;
    }
    private static double normDeg(double a) { double r = a % 360; if (r < 0) r += 360; return r; }
}
