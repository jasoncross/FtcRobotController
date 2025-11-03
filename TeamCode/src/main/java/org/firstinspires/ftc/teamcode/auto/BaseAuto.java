package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
import org.firstinspires.ftc.teamcode.config.FeedTuning;
import org.firstinspires.ftc.teamcode.config.SharedRobotTuning;
import org.firstinspires.ftc.teamcode.config.TeleOpEjectTuning;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/*
 * FILE: BaseAuto.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/
 *
 * PURPOSE
 *   - Provide the shared autonomous scaffold that every alliance-specific class
 *     extends so all DECODE autos rely on identical aiming, driving, and safety
 *     logic.
 *   - Enforce the strategy of "no tag, no fire" by centralizing the lock/aim
 *     checks that protect scoring accuracy.
 *   - Surface telemetry describing the expected start pose so field crews can
 *     double-check orientation each match.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Autonomous pacing & AutoAim)
 *   - SharedRobotTuning.LOCK_TOLERANCE_DEG
 *       • Maximum AprilTag bearing error before declaring a lock.
 *       • Overrides the fallback DEF_LOCK_TOL_DEG below; keep aligned with
 *         Drivebase.TURN_TOLERANCE_DEG for smooth unwinding.
 *   - SharedRobotTuning.TURN_TWIST_CAP
 *       • Clamp for twist power while scanning/aiming.
 *       • Also seeds assist/AutoAimSpeed.maxTwist so TeleOp feels identical.
 *   - SharedRobotTuning.DRIVE_MAX_POWER
 *       • Cap on translational drive helpers (driveForwardInches, strafe, etc.).
 *       • Change here for robot-wide auto speed adjustments instead of editing
 *         individual routines.
 *   - SharedRobotTuning.RPM_TOLERANCE
 *       • Readiness window used by aimSpinUntilReady().
 *       • Coordinate with Launcher.atSpeedToleranceRPM and AutoAimSpeed’s local
 *         copy when adjusting precision.
 *   - SharedRobotTuning.INITIAL_AUTO_DEFAULT_SPEED
 *       • Seeds launcher RPM before the first goal lock via spinLauncherToAutoRpm().
 *       • Mirrors TeleOp's AutoSpeed warm-up so both modes share the same idle spin.
 *   - AutoRpmConfig (NEAR/FAR anchors + SMOOTH_ALPHA)
 *       • Defines the distance→RPM curve applied to LauncherAutoSpeedController.
 *       • Overrides the controller’s internal defaults each time runOpMode() starts.
 *
 * METHODS
 *   - runOpMode()
 *       • Handles subsystem initialization, obelisk observation, and executes
 *         the derived runSequence().
 *   - turnToGoalTag(phase, direction, speed, primarySweepDeg, oppositeSweepDeg)
 *       • Scans for the alliance goal tag using repeatable angular sweeps and
 *         returns once the tuned lock tolerance is satisfied.
 *   - aimSpinUntilReady(timeoutMs)
 *       • Enables AutoSpeed, feeds distance data, and waits for RPM readiness.
 *   - fireN(count)
 *       • Gated shooting loop requiring tag lock + RPM readiness between shots.
 *   - turnBackTo(...), driveForwardInches(...), stopAll(), stopVisionIfAny()
 *       • Utility helpers reused by every derived class.
 *
 * NOTES
 *   - Derived classes must override alliance(), startPoseDescription(), and
 *     runSequence(). Sequence steps can request clockwise or counter-clockwise
 *     sweeps explicitly via {@link AutoSequence#rotateToTarget(String, ScanDirection, double, double, double)}
 *     or the overloads that omit the counter sweep.
 *   - ObeliskSignal captures motif tags during init so autos know which pattern
 *     they are starting in without extra hardware.
 */
public abstract class BaseAuto extends LinearOpMode {

    // CHANGES (2025-10-30): Intake assist now pulls from FeedTuning to reflect tunable relocation.
    // CHANGES (2025-10-31): Added safeInit gating so subsystems stay motionless until START.
    // CHANGES (2025-10-31): Added unified telemetry/status surface, live obelisk refresh, and
    //                        stricter lock-at-speed visibility for Auto volley orchestration.
    // CHANGES (2025-10-31): Synced auto start/stop behavior with TeleOp defaults (intake on at START,
    //                        feed idle hold engaged only during run, released on stopAll()).
    // CHANGES (2025-10-31): aimSpinUntilReady() now seeds launcher RPM exclusively through AutoSpeed so
    //                        autos honor AutoRpmConfig defaults prior to the first tag lock.
    // CHANGES (2025-10-31): Introduced AutoSequence builder for declarative route scripting and added
    //                        optional no-lock volleys plus customizable scan/aim status labels.
    // CHANGES (2025-10-31): updateStatus now mirrors AutoSequence labels on a dedicated telemetry line so
    //                        Driver Station shows each step description during execution.
    // CHANGES (2025-11-02): Added spinLauncherToAutoRpm() warm-up helper, parameterized fire cadence,
    //                        reasserted AutoSpeed targets during volleys, and extended AutoSequence to
    //                        pre-spin flywheels ahead of tag locks.

    // Implemented by child classes to define alliance, telemetry description, scan direction, and core actions.
    protected abstract Alliance alliance();
    protected abstract String startPoseDescription();
    /** Direction helpers for tag scan sweeps. */
    public enum ScanDirection {
        CW,
        CCW;

        private boolean isClockwise() {
            return this == CW;
        }
    }

    /** State machine phases used by {@link #turnToGoalTag} while sweeping for the goal tag. */
    private enum SweepState {
        PRIMARY_OUT,
        RETURN_TO_ZERO,
        OPPOSITE_OUT,
        OPPOSITE_RETURN,
        PARTIAL_RETURN,
        HOLD_PRIMARY
    }
    protected abstract void runSequence() throws InterruptedException;

    // Optional hook allowing derived autos to perform extra telemetry or sensor prep pre-start.
    protected void onPreStartLoop() {}

    // Core subsystems shared by all autos.
    protected Drivebase drive;           // Field-centric mecanum drive helper
    protected VisionAprilTag vision;     // AprilTag pipeline (goal + obelisk)
    protected Launcher launcher;         // Flywheel subsystem for scoring
    protected Feed feed;                 // Artifact feed motor controller
    protected Intake intake;             // Intake roller subsystem

    // Controllers supporting aiming and RPM automation.
    protected final TagAimController aim = new TagAimController();
    protected final LauncherAutoSpeedController autoCtrl = new LauncherAutoSpeedController();

    // Local defaults if config fails (protects against missing SharedRobotTuning definitions).
    private static final double DEF_LOCK_TOL_DEG   = 1.0;
    private static final double DEF_TURN_CAP       = 0.35;
    private static final double DEF_DRIVE_CAP      = 0.50;
    private static final double DEF_RPM_TOL        = 50.0;
    private static final double DEF_AUTO_SEED_RPM  = 2500.0;
    private static final long   DEFAULT_BETWEEN_MS = 3000; // Default between-shot wait used when callers pass ≤ 0

    private String autoOpModeName;

    private double lockTolDeg()   { try { return SharedRobotTuning.LOCK_TOLERANCE_DEG; } catch (Throwable t){ return DEF_LOCK_TOL_DEG; } }
    private double turnTwistCap() { try { return SharedRobotTuning.TURN_TWIST_CAP;     } catch (Throwable t){ return DEF_TURN_CAP;     } }
    private double driveCap()     { try { return SharedRobotTuning.DRIVE_MAX_POWER;    } catch (Throwable t){ return DEF_DRIVE_CAP;    } }
    private double rpmTol()       { try { return SharedRobotTuning.RPM_TOLERANCE;      } catch (Throwable t){ return DEF_RPM_TOL;      } }
    private double autoSeedRpm() {
        try { return SharedRobotTuning.INITIAL_AUTO_DEFAULT_SPEED; }
        catch (Throwable t) { return DEF_AUTO_SEED_RPM; }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        autoOpModeName = resolveAutoName();

        // Create drivetrain with IMU + encoder helpers for field-aligned movement.
        drive = new Drivebase(this);
        // Initialize AprilTag vision; guard against missing camera on practice bot.
        try { vision = new VisionAprilTag(); vision.init(hardwareMap, "Webcam 1"); } catch (Exception ex) { vision = null; }
        launcher = new Launcher(hardwareMap);   // Flywheel pair
        feed     = new Feed(hardwareMap);       // Indexer wheel
        intake   = new Intake(hardwareMap);     // Floor intake

        // Guarantee INIT remains motionless until START.
        drive.safeInit();
        launcher.safeInit();
        feed.safeInit();
        intake.safeInit();

        try { AutoRpmConfig.apply(autoCtrl); } catch (Throwable ignored) {} // Sync AutoSpeed curve
        ObeliskSignal.clear(); // Reset Obelisk latch before looking for motifs

        while (!isStarted() && !isStopRequested()) {
            updateStatus("INIT", false);
            onPreStartLoop();
            telemetry.update();
            sleep(20);
        }
        if (isStopRequested()) { stopVisionIfAny(); return; }
        feed.setIdleHoldActive(true); // Allow idle counter-rotation only after START
        intake.set(true);             // Mirror TeleOp default: intake runs once the match starts
        if (vision != null) vision.setObeliskAutoLatchEnabled(true); // Capture motifs during movement

        try { runSequence(); }
        finally {
            stopAll();
            stopVisionIfAny();
            updateStatus("COMPLETE", false);
            telemetry.addLine("Auto complete – DS will queue TeleOp."); telemetry.update();
            sleep(250);
        }
    }

    // ---------- Tag search/center ----------
    /**
     * Rotate until the alliance goal AprilTag is within tolerance or timeout occurs.
     * Returns true when lock achieved, false when timed out.
     */
    /**
     * Search for the alliance goal tag by sweeping around the current heading until the
     * AprilTag lock tolerance is satisfied.
     *
     * @param phase              Telemetry label shown while scanning (defaults to
     *                           "Scan for goal tag" when empty).
     * @param direction          Opening sweep direction. Pass {@code null} to default to clockwise.
     * @param turnSpeedFraction  Fraction (0–1] of {@link SharedRobotTuning#TURN_TWIST_CAP} to use while scanning.
     * @param primarySweepDeg    Degrees to sweep in the opening direction before returning through center.
     * @param oppositeSweepDeg   How far to probe past center on the counter sweep. Positive values travel through
     *                           zero into the opposite direction by the requested magnitude. Negative values stop
     *                           before reaching zero by the requested magnitude (relative to the opening side).
     *                           Pass {@code 0} to return to center before repeating, or {@code null} / {@link Double#NaN}
     *                           to hold at the primary sweep limit with no counter pass.
     */
    protected final boolean turnToGoalTag(String phase,
                                           ScanDirection direction,
                                           double turnSpeedFraction,
                                           double primarySweepDeg,
                                           Double oppositeSweepDeg) {
        final int goalId = (alliance() == Alliance.BLUE) ? VisionAprilTag.TAG_BLUE_GOAL : VisionAprilTag.TAG_RED_GOAL;
        final double tol = lockTolDeg();
        final double cap = turnTwistCap();
        final String label = (phase == null || phase.isEmpty()) ? "Scan for goal tag" : phase;

        final double speedFrac = Math.max(0.05, Math.min(Math.abs(turnSpeedFraction), 1.0));
        final double twist = cap * speedFrac;
        final double primaryLimit = Math.max(1.0, Math.abs(primarySweepDeg));
        final boolean disableOpposite = (oppositeSweepDeg == null) || Double.isNaN(oppositeSweepDeg);
        final double rawOpposite = disableOpposite ? 0.0 : oppositeSweepDeg;
        final double oppositeLimit = Math.abs(rawOpposite);
        final boolean oppositeCrossesZero = !disableOpposite && rawOpposite > 0.0;
        final boolean oppositeStaysSameSide = !disableOpposite && rawOpposite < 0.0;
        final double zeroTol = 1.5; // degrees around the neutral heading treated as "zero"

        ScanDirection resolved = (direction == null) ? ScanDirection.CW : direction;
        double primarySign = resolved.isClockwise() ? -1.0 : +1.0; // CW scanning uses negative twist
        double primaryTarget = primarySign * primaryLimit;
        double oppositeTarget = 0.0;
        double partialTarget = 0.0;
        if (oppositeCrossesZero) {
            double magnitude = Math.max(1.0, oppositeLimit);
            oppositeTarget = -primarySign * magnitude;
        } else if (oppositeStaysSameSide) {
            double magnitude = Math.max(1.0, Math.min(primaryLimit, oppositeLimit));
            partialTarget = primarySign * magnitude;
        }

        double zeroHeading = drive.heading();
        final String sweepSummary;
        if (disableOpposite) {
            sweepSummary = String.format(Locale.US, "%.1f / --", primaryTarget);
        } else if (oppositeCrossesZero) {
            sweepSummary = String.format(Locale.US, "%.1f / %.1f", primaryTarget, oppositeTarget);
        } else if (oppositeStaysSameSide) {
            sweepSummary = String.format(Locale.US, "%.1f / %.1f", primaryTarget, partialTarget);
        } else {
            sweepSummary = String.format(Locale.US, "%.1f / %.1f", primaryTarget, 0.0);
        }

        SweepState state = SweepState.PRIMARY_OUT;

        while (opModeIsActive()) {
            AprilTagDetection det = (vision != null) ? vision.getDetectionFor(goalId) : null;
            double bearing = Double.NaN;
            boolean lockedNow = false;

            if (det != null) {
                double err = det.ftcPose.bearing;
                bearing = err;
                double cmd = clamp(aim.turnPower(det), -cap, +cap);
                drive.drive(0, 0, cmd);
                lockedNow = Math.abs(err) <= tol;
                if (lockedNow) {
                    drive.stopAll();
                    updateStatus(label + " – lock", true);
                    telemetry.addData("Bearing (deg)", err);
                    telemetry.update();
                    return true;
                }
            } else {
                double offset = shortestDiff(drive.heading(), zeroHeading);
                double command = 0.0;

                switch (state) {
                    case PRIMARY_OUT:
                        command = primarySign * twist;
                        if ((primarySign < 0 && offset <= primaryTarget) ||
                                (primarySign > 0 && offset >= primaryTarget)) {
                            if (disableOpposite) {
                                state = SweepState.HOLD_PRIMARY;
                            } else {
                                state = oppositeStaysSameSide ? SweepState.PARTIAL_RETURN : SweepState.RETURN_TO_ZERO;
                            }
                        }
                        break;
                    case RETURN_TO_ZERO:
                        command = -primarySign * twist;
                        if (Math.abs(offset) <= zeroTol) {
                            zeroHeading = drive.heading();
                            state = oppositeCrossesZero ? SweepState.OPPOSITE_OUT : SweepState.PRIMARY_OUT;
                        }
                        break;
                    case OPPOSITE_OUT:
                        command = -primarySign * twist;
                        if ((-primarySign < 0 && offset <= oppositeTarget) ||
                                (-primarySign > 0 && offset >= oppositeTarget)) {
                            state = SweepState.OPPOSITE_RETURN;
                        }
                        break;
                    case OPPOSITE_RETURN:
                        command = primarySign * twist;
                        if (Math.abs(offset) <= zeroTol) {
                            zeroHeading = drive.heading();
                            state = SweepState.PRIMARY_OUT;
                        }
                        break;
                    case PARTIAL_RETURN:
                        command = -primarySign * twist;
                        if (primarySign > 0) {
                            if (offset <= partialTarget) {
                                state = SweepState.PRIMARY_OUT;
                            }
                        } else {
                            if (offset >= partialTarget) {
                                state = SweepState.PRIMARY_OUT;
                            }
                        }
                        break;
                    case HOLD_PRIMARY:
                        command = 0.0;
                        break;
                }

                drive.drive(0, 0, command);
                telemetry.addData("Scan offset (deg)", offset);
                telemetry.addData("Scan state", state);
            }

            updateStatus(label, lockedNow);
            telemetry.addData("Bearing (deg)", bearing);
            telemetry.addData("Turn speed (|twist|)", twist);
            telemetry.addData("Sweep offsets (deg)", sweepSummary);
            telemetry.update();
            idle();
        }
        drive.stopAll();
        updateStatus(label + " – cancelled", false);
        telemetry.update();
        return false;
    }

    // ---------- Spin to at-speed ----------
    /**
     * Enable AutoSpeed, feed AprilTag distance into the curve, and wait for launcher RPM
     * to enter the tuned tolerance window. Returns true when ready before timeout.
     */
    protected final boolean aimSpinUntilReady(long timeoutMs) {
        return aimSpinUntilReady(timeoutMs, "Spin launcher");
    }

    protected final boolean aimSpinUntilReady(long timeoutMs, String phase) {
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

            double target = autoCtrl.updateWithVision(in);
            launcher.setTargetRpm(target);

            boolean atSpeed = Math.abs(launcher.getCurrentRpm() - launcher.targetRpm) <= rpmTol();
            updateStatus(phase, hadFix && in != null);
            telemetry.addData("Target RPM", launcher.targetRpm);
            telemetry.addData("Current RPM", launcher.getCurrentRpm());
            telemetry.addData("Range (in)", in);
            telemetry.update();
            if (atSpeed) {
                return true;
            }
            idle();
        }
        updateStatus(phase + " – timeout", false);
        telemetry.addData("Target RPM", launcher.targetRpm);
        telemetry.addData("Current RPM", launcher.getCurrentRpm());
        telemetry.update();
        return false;
    }

    // ---------- Strictly gated shooting ----------
    /**
     * Fire count artifacts with strict gating—requires tag lock for each shot and enforces
     * both RPM readiness and the caller-supplied between-shot delay.
     */
    protected final void fireN(int count) throws InterruptedException {
        fireN(count, true, DEFAULT_BETWEEN_MS);
    }

    protected final void fireN(int count, boolean requireLock) throws InterruptedException {
        fireN(count, requireLock, DEFAULT_BETWEEN_MS);
    }

    protected final void fireN(int count, boolean requireLock, long betweenShotsMs) throws InterruptedException {
        autoCtrl.setAutoEnabled(true);
        for (int i = 0; i < count && opModeIsActive(); i++) {
            final String shotPhase = String.format("Volley %d/%d", i + 1, count);
            boolean lockedForShot = !requireLock;
            if (requireLock) {
                lockedForShot = requireLockOrTimeOut(1200, shotPhase + " – acquire lock");
                if (!lockedForShot) {
                    updateStatus("Hold position", false);
                    telemetry.addLine("⚠️ No tag lock — skipping shot " + (i + 1));
                    telemetry.update();
                    continue; // do not free-fire when lock required
                }
            }

            double holdTarget = autoCtrl.hold();
            if (holdTarget <= 0) {
                holdTarget = launcher.targetRpm;
                if (holdTarget <= 0) {
                    holdTarget = autoSeedRpm();
                }
            }
            launcher.setTargetRpm(holdTarget);

            // REQUIRE at-speed
            while (opModeIsActive()) {
                updateStatus(shotPhase + " – wait for RPM", lockedForShot || !requireLock);
                telemetry.addData("Target RPM", launcher.targetRpm);
                telemetry.addData("Current RPM", launcher.getCurrentRpm());
                telemetry.update();
                if (Math.abs(launcher.getCurrentRpm() - launcher.targetRpm) <= rpmTol()) break;
                idle();
            }

            // Feed once with intake assist
            boolean wasOn = intake.isOn();
            if (!wasOn) intake.set(true);
            updateStatus(shotPhase + " – feed", lockedForShot || !requireLock);
            telemetry.addData("Target RPM", launcher.targetRpm);
            telemetry.addData("Current RPM", launcher.getCurrentRpm());
            telemetry.update();
            feed.feedOnceBlocking();
            if (!wasOn) {
                int assist = FeedTuning.INTAKE_ASSIST_MS;
                sleep(assist);
                intake.set(false);
            }

            // Reassert the AutoSpeed target so the flywheels stay at commanded RPM during recovery.
            double recoverTarget = autoCtrl.hold();
            if (recoverTarget <= 0) {
                recoverTarget = holdTarget;
            }
            launcher.setTargetRpm(recoverTarget);

            long delay = (betweenShotsMs > 0) ? betweenShotsMs : DEFAULT_BETWEEN_MS;
            sleep((int)delay);
            drive.stopAll();
            updateStatus("Stabilize after volley", lockedForShot || !requireLock);
            telemetry.update();
        }
    }

    /** Wait up to guardMs to achieve a tag lock (|bearing| ≤ tol). Returns true if locked. */
    private boolean requireLockOrTimeOut(long guardMs, String phase) {
        return requireLockOrTimeOut(guardMs, phase, (ScanDirection) null);
    }

    private boolean requireLockOrTimeOut(long guardMs, String phase, Boolean scanClockwiseFirst) {
        ScanDirection dir = (scanClockwiseFirst == null)
                ? null
                : (scanClockwiseFirst ? ScanDirection.CW : ScanDirection.CCW);
        return requireLockOrTimeOut(guardMs, phase, dir);
    }

    private boolean requireLockOrTimeOut(long guardMs, String phase, ScanDirection direction) {
        final int goalId = (alliance() == Alliance.BLUE) ? VisionAprilTag.TAG_BLUE_GOAL : VisionAprilTag.TAG_RED_GOAL;
        final double tol = lockTolDeg();
        final double cap = turnTwistCap();
        final String label = (phase == null || phase.isEmpty()) ? "Acquire lock" : phase;
        ScanDirection resolved = (direction == null) ? ScanDirection.CW : direction;
        boolean cwFirst = resolved.isClockwise();
        double scanSign = cwFirst ? -1.0 : +1.0;
        long start = System.currentTimeMillis();
        long lastFlip = start;

        while (opModeIsActive() && (System.currentTimeMillis() - start) < guardMs) {
            AprilTagDetection det = (vision != null) ? vision.getDetectionFor(goalId) : null;
            if (det != null) {
                double err = det.ftcPose.bearing;
                boolean locked = Math.abs(err) <= tol;
                if (locked) {
                    drive.stopAll();
                    updateStatus(label, true);
                    telemetry.addData("Bearing (deg)", err);
                    telemetry.update();
                    return true;
                }
                double cmd = clamp(aim.turnPower(det), -cap, +cap);
                drive.drive(0, 0, cmd * 0.6);
                updateStatus(label, false);
                telemetry.addData("Bearing (deg)", err);
            } else {
                long now = System.currentTimeMillis();
                if (now - lastFlip > 600) { scanSign *= -1.0; lastFlip = now; }
                drive.drive(0, 0, scanSign * 0.25 * cap);
                updateStatus(label, false);
                telemetry.addData("Bearing (deg)", Double.NaN);
            }
            telemetry.update();
            idle();
        }
        drive.stopAll();
        updateStatus(label + " – timeout", false);
        telemetry.update();
        return false;
    }

    /** Command an IMU-based turn back to the stored heading. */
    protected final void turnBackTo(double startHeadingDeg) {
        double cur = drive.heading();
        double delta = shortestDiff(startHeadingDeg, cur);
        drive.turn(delta, clamp(turnTwistCap() + 0.05, 0.2, 0.8));
    }

    /** Drive straight forward using the tuned drive cap. */
    protected final void driveForwardInches(double inches) {
        drive.move(inches, 0.0, driveCap());
    }

    protected final void driveForwardInches(double inches, double speedCap) {
        drive.move(inches, 0.0, clampTranslationSpeed(speedCap));
    }

    protected final void turnToHeading(double headingDeg, double speedCap) {
        double cur = drive.heading();
        double delta = shortestDiff(headingDeg, cur);
        drive.turn(delta, clampTurnSpeed(speedCap));
    }

    /** Stop all active subsystems (safety catch-all). */
    protected final void stopAll() {
        try { drive.stop(); } catch (Throwable ignored) {}
        try { launcher.stop(); } catch (Throwable ignored) {}
        try { feed.setIdleHoldActive(false); feed.stop(); } catch (Throwable ignored) {}
        try { intake.stop(); } catch (Throwable ignored) {}
        try { autoCtrl.setAutoEnabled(false); } catch (Throwable ignored) {}
    }
    /** Shutdown the vision portal safely if it was created. */
    protected final void stopVisionIfAny() {
        try { if (vision != null) { vision.setObeliskAutoLatchEnabled(false); vision.stop(); } } catch (Exception ignored) {}
    }

    private String resolveAutoName() {
        Autonomous meta = getClass().getAnnotation(Autonomous.class);
        if (meta != null) {
            String name = meta.name();
            if (name != null && !name.isEmpty()) {
                return name;
            }
        }
        return getClass().getSimpleName();
    }

    protected final void updateStatus(String phase, boolean tagLocked) {
        String displayPhase = (phase == null) ? "" : phase.trim();
        if (displayPhase.isEmpty()) {
            displayPhase = "(no label)";
        }
        if (vision != null) {
            try { vision.observeObelisk(); } catch (Throwable ignored) {}
        }
        telemetry.addData("Alliance", alliance());
        telemetry.addData("Auto", autoOpModeName);
        telemetry.addData("Start Pose", startPoseDescription());
        telemetry.addData("Obelisk", ObeliskSignal.getDisplay());
        telemetry.addData("AprilTag Lock", tagLocked ? "LOCKED" : "SEARCHING");
        telemetry.addData("Phase", displayPhase);
        telemetry.addData("Sequence Step", displayPhase);
    }

    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
    private static double shortestDiff(double target, double current) {
        double d = normDeg(target - current); if (d > 180) d -= 360; if (d < -180) d += 360; return d;
    }
    private static double normDeg(double a) { double r = a % 360; if (r < 0) r += 360; return r; }

    private double clampTranslationSpeed(double requested) {
        double cap = driveCap();
        if (requested <= 0) { return cap; }
        return Math.min(requested, cap);
    }

    private double clampTurnSpeed(double requested) {
        double defaultCap = clamp(turnTwistCap() + 0.1, 0.2, 0.8);
        if (requested <= 0) { return defaultCap; }
        return clamp(requested, 0.2, Math.max(0.2, defaultCap));
    }

    protected final void spinLauncherToAutoRpm(String phase) {
        String label = (phase == null || phase.isEmpty()) ? "Spin to auto RPM" : phase;
        drive.stopAll();
        autoCtrl.setAutoEnabled(true);
        double seed = autoSeedRpm();
        try { autoCtrl.setDefaultRpm(seed); } catch (Throwable ignored) {}
        double target = autoCtrl.hold();
        if (target <= 0) { target = seed; }
        launcher.setTargetRpm(target);
        updateStatus(label, false);
        telemetry.addData("Target RPM", target);
        telemetry.addData("Auto default RPM", seed);
        telemetry.update();
    }

    protected final AutoSequence sequence() {
        return new AutoSequence();
    }

    protected final class AutoSequence {
        private final List<AutoStep> steps = new ArrayList<>();
        private double storedHeading = Double.NaN;
        private boolean lastLock = false;
        private boolean lastAimReady = false;

        private AutoSequence addStep(AutoStep step) {
            steps.add(step);
            return this;
        }

        private String resolveLabel(String provided, String fallback) {
            return (provided == null || provided.isEmpty()) ? fallback : provided;
        }

        public AutoSequence rememberHeading(String phase) {
            return addStep(() -> {
                storedHeading = drive.heading();
                String label = resolveLabel(phase, "Record heading");
                updateStatus(label, lastLock);
                telemetry.addData("Stored heading (deg)", storedHeading);
                telemetry.update();
            });
        }

        public AutoSequence move(String phase, double distanceInches, double headingDeg, double speedCap) {
            return addStep(() -> {
                String label = resolveLabel(phase, "Move");
                lastLock = false;
                lastAimReady = false;
                double speed = clampTranslationSpeed(speedCap);
                updateStatus(label, false);
                telemetry.addData("Distance (in)", distanceInches);
                telemetry.addData("Heading (deg)", headingDeg);
                telemetry.addData("Speed cap", speed);
                telemetry.update();
                drive.move(distanceInches, headingDeg, speed);
                drive.stopAll();
                updateStatus(label + " complete", false);
                telemetry.update();
            });
        }

        public AutoSequence rotate(String phase, double degrees, double speedCap) {
            return addStep(() -> {
                String label = resolveLabel(phase, "Rotate");
                lastLock = false;
                lastAimReady = false;
                double speed = clampTurnSpeed(speedCap);
                updateStatus(label, false);
                telemetry.addData("Delta (deg)", degrees);
                telemetry.addData("Speed cap", speed);
                telemetry.update();
                drive.turn(degrees, speed);
                updateStatus(label + " complete", false);
                telemetry.update();
            });
        }

        public AutoSequence rotateToHeading(String phase, double headingDeg, double speedCap) {
            return addStep(() -> {
                String label = resolveLabel(phase, "Rotate to heading");
                lastLock = false;
                lastAimReady = false;
                double speed = clampTurnSpeed(speedCap);
                updateStatus(label, false);
                telemetry.addData("Target heading (deg)", headingDeg);
                telemetry.addData("Speed cap", speed);
                telemetry.update();
                double delta = shortestDiff(headingDeg, drive.heading());
                drive.turn(delta, speed);
                updateStatus(label + " complete", false);
                telemetry.update();
            });
        }

        public AutoSequence spinToAutoRpm(String phase) {
            return addStep(() -> {
                lastLock = false;
                lastAimReady = false;
                spinLauncherToAutoRpm(phase);
            });
        }

        public AutoSequence rotateToTarget(String phase,
                                           ScanDirection direction,
                                           double turnSpeedFraction,
                                           double primarySweepDeg,
                                           double oppositeSweepDeg) {
            return addStep(() -> {
                lastAimReady = false;
                lastLock = turnToGoalTag(phase, direction, turnSpeedFraction, primarySweepDeg, oppositeSweepDeg);
                if (!lastLock) {
                    telemetry.addLine("⚠️ No tag lock – continuing sequence");
                    telemetry.update();
                }
            });
        }

        public AutoSequence rotateToTarget(String phase,
                                           double turnSpeedFraction,
                                           double primarySweepDeg,
                                           double oppositeSweepDeg) {
            return rotateToTarget(phase, null, turnSpeedFraction, primarySweepDeg, oppositeSweepDeg);
        }

        public AutoSequence rotateToTarget(String phase,
                                           ScanDirection direction,
                                           double turnSpeedFraction,
                                           double primarySweepDeg) {
            return addStep(() -> {
                lastAimReady = false;
                lastLock = turnToGoalTag(phase, direction, turnSpeedFraction, primarySweepDeg, null);
                if (!lastLock) {
                    telemetry.addLine("⚠️ No tag lock – continuing sequence");
                    telemetry.update();
                }
            });
        }

        public AutoSequence rotateToTarget(String phase,
                                           double turnSpeedFraction,
                                           double primarySweepDeg) {
            return rotateToTarget(phase, null, turnSpeedFraction, primarySweepDeg);
        }

        public AutoSequence intake(String phase, boolean enabled) {
            return addStep(() -> {
                String label = resolveLabel(phase, enabled ? "Enable intake" : "Disable intake");
                lastLock = false;
                lastAimReady = false;
                updateStatus(label, false);
                telemetry.addData("Intake state", enabled ? "ON" : "OFF");
                telemetry.update();
                intake.set(enabled);
            });
        }

        public AutoSequence stop(String phase) {
            return addStep(() -> {
                String label = resolveLabel(phase, "Stop all");
                lastLock = false;
                lastAimReady = false;
                updateStatus(label, false);
                telemetry.update();
                stopAll();
                updateStatus(label + " complete", false);
                telemetry.update();
            });
        }

        public AutoSequence aim(String phase, long timeoutMs) {
            return addStep(() -> {
                lastAimReady = aimSpinUntilReady(timeoutMs, phase);
                if (!lastAimReady) {
                    telemetry.addLine("⚠️ Launcher not at speed before timeout");
                    telemetry.update();
                }
            });
        }

        public AutoSequence waitFor(String phase, long milliseconds) {
            return addStep(() -> {
                String label = resolveLabel(phase, "Wait");
                updateStatus(label, lastLock);
                telemetry.addData("Duration (ms)", milliseconds);
                telemetry.update();
                sleep(milliseconds);
            });
        }

        public AutoSequence fire(String phase, int shots, boolean requireLock, long betweenShotsMs) {
            return addStep(() -> {
                String label = resolveLabel(phase, "Fire");
                if (requireLock && !lastLock) {
                    updateStatus(label + " – skipped (no lock)", false);
                    telemetry.addLine("⚠️ Fire skipped because tag lock was not achieved");
                    telemetry.update();
                    return;
                }
                updateStatus(label, !requireLock || lastLock);
                telemetry.addData("Shots", shots);
                telemetry.addData("Lock required", requireLock);
                telemetry.addData("Between shots (ms)", betweenShotsMs);
                telemetry.update();
                fireN(shots, requireLock, betweenShotsMs);
                lastAimReady = false;
                lastLock = requireLock && lastLock;
            });
        }

        public AutoSequence eject(String phase) {
            return addStep(() -> {
                String label = resolveLabel(phase, "Eject artifact");
                lastLock = false;
                lastAimReady = false;

                updateStatus(label, false);
                telemetry.addData("Eject RPM", TeleOpEjectTuning.RPM);
                telemetry.addData("Eject duration (ms)", TeleOpEjectTuning.TIME_MS);
                telemetry.update();

                double previousTarget = launcher.targetRpm;
                double previousHold = autoCtrl.hold();
                boolean autoWasEnabled = autoCtrl.isAutoEnabled();
                if (autoWasEnabled) {
                    double manualSync = previousTarget;
                    if (manualSync <= 0 && previousHold > 0) {
                        manualSync = previousHold;
                    }
                    autoCtrl.onManualOverride(manualSync);
                }

                int ejectDuration = Math.max(100, TeleOpEjectTuning.TIME_MS);
                int spinUpDelay = Math.max(100, ejectDuration / 3);

                double ejectRpm = Math.max(0.0, TeleOpEjectTuning.RPM);
                launcher.setTargetRpm(ejectRpm);
                sleep(spinUpDelay);

                boolean intakeWasOn = intake.isOn();
                if (!intakeWasOn) intake.set(true);
                feed.feedOnceBlocking();
                sleep(ejectDuration);
                if (!intakeWasOn) {
                    sleep(FeedTuning.INTAKE_ASSIST_MS);
                    intake.set(false);
                }

                double restoreRpm = previousTarget;
                if (autoWasEnabled) {
                    autoCtrl.setAutoEnabled(true);
                    double holdTarget = previousHold;
                    if (holdTarget <= 0) {
                        holdTarget = (restoreRpm > 0) ? restoreRpm : autoSeedRpm();
                    }
                    launcher.setTargetRpm(holdTarget);
                } else {
                    launcher.setTargetRpm(restoreRpm);
                }

                updateStatus(label + " complete", false);
                telemetry.update();
            });
        }

        public AutoSequence returnToStoredHeading(String phase, double speedCap) {
            return addStep(() -> {
                double target = Double.isNaN(storedHeading) ? drive.heading() : storedHeading;
                String label = resolveLabel(phase, "Return to heading");
                lastLock = false;
                lastAimReady = false;
                double speed = clampTurnSpeed(speedCap);
                updateStatus(label, false);
                telemetry.addData("Target heading (deg)", target);
                telemetry.addData("Speed cap", speed);
                telemetry.update();
                double delta = shortestDiff(target, drive.heading());
                drive.turn(delta, speed);
                updateStatus(label + " complete", false);
                telemetry.update();
            });
        }

        public AutoSequence custom(AutoStep step) {
            return addStep(step);
        }

        public void run() throws InterruptedException {
            for (AutoStep step : steps) {
                if (!opModeIsActive()) {
                    break;
                }
                step.run();
            }
        }
    }

    @FunctionalInterface
    protected interface AutoStep {
        void run() throws InterruptedException;
    }
}
