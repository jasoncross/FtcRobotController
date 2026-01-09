package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.config.AutoAimTuning;
import org.firstinspires.ftc.teamcode.config.AutoRpmConfig;
import org.firstinspires.ftc.teamcode.config.DebugTelemetryConfig;
import org.firstinspires.ftc.teamcode.config.FeedTuning;
import org.firstinspires.ftc.teamcode.config.OdometryConfig;
import org.firstinspires.ftc.teamcode.config.SharedRobotTuning;
import org.firstinspires.ftc.teamcode.config.TeleOpDriverDefaults;
import org.firstinspires.ftc.teamcode.config.TeleOpEjectTuning;
import org.firstinspires.ftc.teamcode.config.VisionConfig;
import org.firstinspires.ftc.teamcode.config.VisionTuning;
import org.firstinspires.ftc.teamcode.control.LauncherAutoSpeedController;
import org.firstinspires.ftc.teamcode.control.FiringController;
import org.firstinspires.ftc.teamcode.drive.Drivebase;
import org.firstinspires.ftc.teamcode.odometry.DecodeFieldDrawing;
import org.firstinspires.ftc.teamcode.odometry.FieldPose;
import org.firstinspires.ftc.teamcode.odometry.Odometry;
import org.firstinspires.ftc.teamcode.odometry.PoseStore;
import org.firstinspires.ftc.teamcode.subsystems.Feed;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.utils.ObeliskSignal;
import org.firstinspires.ftc.teamcode.vision.LimelightPipelineAutoSelector;
import org.firstinspires.ftc.teamcode.vision.LimelightTargetProvider;
import org.firstinspires.ftc.teamcode.vision.TagAimController;
import org.firstinspires.ftc.teamcode.vision.VisionAprilTag;
import org.firstinspires.ftc.teamcode.vision.VisionTargetProvider;
import org.firstinspires.ftc.teamcode.vision.WebcamLegacyTargetProvider;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Objects;

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
 *       • Seeds launcher RPM before the first goal lock via spinLauncherToAutoRpmDefault().
 *       • Mirrors TeleOp's AutoSpeed warm-up so both modes share the same idle spin.
 *   - AutoRpmConfig (calibration table + SMOOTH_ALPHA)
 *       • Defines the ordered distance→RPM calibration points for LauncherAutoSpeedController.
 *       • Overrides the controller’s internal defaults each time runOpMode() starts.
 *
 * METHODS
 *   - runOpMode()
 *       • Handles subsystem initialization, obelisk observation, and executes
 *         the derived runSequence().
 *   - turnToGoalTag(phase, direction, speed, primarySweepDeg, oppositeSweepDeg)
 *       • Scans for the alliance goal tag using repeatable angular sweeps and
 *         returns once the tuned lock tolerance is satisfied.
 *   - readyLauncherUntilReady(timeoutMs)
 *       • Enables AutoSpeed, feeds distance data, waits for settle time inside the RPM window.
 *   - fireN(count)
 *       • Gated shooting loop requiring tag lock + RPM readiness between shots.
 *   - turnBackTo(...), driveForwardInches(...), stopAll(), stopVisionIfAny()
 *       • Utility helpers reused by every derived class.
 *
 * NOTES
 *   - Derived classes must override alliance(), startPoseDescription(), and
 *     runSequence(). Sequence steps can request clockwise or counter-clockwise
 *     sweeps explicitly via {@link AutoSequence#rotateToTarget(String, ScanDirection, double, double, double, long)}
 *     or the overloads that omit the counter sweep while still requiring a timeout.
 *   - ObeliskSignal captures motif tags during init so autos know which pattern
 *     they are starting in without extra hardware.
 */
public abstract class BaseAuto extends LinearOpMode {

    // CHANGES (2025-12-28): Continued Limelight pipeline auto-selection after START,
    //                        removed START-forced fallback so selection can lock
    //                        post-start, and kept INIT selection running through
    //                        START without resets.
    // CHANGES (2025-12-28): Added Limelight pipeline memory fallback telemetry
    //                        lines while preserving urgent failure banner ordering.
    // CHANGES (2026-01-09): Synced the auto-start intake enable with the firing controller
    //                        desired state so all autos begin with intake running.
    // CHANGES (2026-01-09): Reasserted intake enable after auto firing sequences so intake
    //                        resumes automatically once volleys complete.
    // CHANGES (2026-01-07): Added debug-only firing telemetry lines for feedstop
    //                        and cadence profiling during auto sequences.
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
    // CHANGES (2025-11-02): Added spinLauncherToAutoRpm() warm-up helper, parameterized fire cadence,
    //                        reasserted AutoSpeed targets during volleys, and extended AutoSequence to
    //                        pre-spin flywheels ahead of tag locks.
    // CHANGES (2025-11-03): Raised sequence labels to the top of telemetry with spacing so active phases
    //                        stay visible while extra status lines append beneath them.
    // CHANGES (2025-11-03): Renamed aim() → readyToLaunch(), added RPM settle gating, and matched the
    //                        AutoSpeed calibration flow used by TeleOp for launcher prep.
    // CHANGES (2025-11-04): stopAll() now latches BRAKE zero-power behavior across subsystems for end-of-match hold.
    // CHANGES (2025-11-05): Applied VisionTuning range scale during Auto init and added
    //                        AutoSequence.visionMode(...) for mid-sequence AprilTag profile swaps.
    // CHANGES (2025-11-07): Updated fireN() to rely on Feed.beginFeedCycle() so the new
    //                        asynchronous servo lead applies consistently without manual release calls.
    // CHANGES (2025-11-07): Surfaced FeedStop homing telemetry so INIT shows when zero is established.
    // CHANGES (2025-12-18): Applied AutoAim twist inversion to AUTO drive commands so aim direction matches the TeleOp tunable.
    // CHANGES (2025-12-16): Reversed long-shot lock bias so RED uses negative bearings
    //                        and BLUE uses positive bearings when long-range aim windows engage.
    // CHANGES (2025-11-09): Trimmed FeedStop telemetry to a single summary line with warnings only
    //                        when soft-limit or scaling guards trigger.
    // CHANGES (2025-11-14): Relaxed AprilTag lock tolerance automatically when the 480p vision
    //                        profile is active so coarse pose noise no longer stalls volleys.
    // CHANGES (2025-11-15): Documented the AutoRpmConfig calibration table so autos follow the
    //                        same multi-point curve as TeleOp after apply().
    // CHANGES (2025-11-16): Wired the encoder-aware intake flow updater into every blocking loop so
    //                        autos maintain the same jam protection while aiming or waiting.
    // CHANGES (2025-11-22): Added a tunable master toggle for long-shot biasing to restore symmetric
    //                        lock windows without changing alliance-specific logic.
    // CHANGES (2026-01-03): Added requireLauncherAtSpeed parameters for AutoSequence firing
    //                        steps so autos can choose RPM gating per action.
    // CHANGES (2026-01-05): Wired the launcher readiness latch into auto firing loops so
    //                        fast-path shots and recovery timing stay consistent.
    // CHANGES (2025-11-18): Biased the tag lock window toward alliance-correct angles when
    //                        the robot is beyond the long-shot distance cutover.
    // CHANGES (2025-11-26): Persist the final fused odometry pose with explicit status-aware
    //                        pose store helpers so TeleOp can resume from the last Auto pose
    //                        when no AprilTag seed is available at INIT.
    // CHANGES (2025-11-25): rotateToTarget steps still require explicit timeouts per sequence,
    //                        and autos now inline their own 10 s default instead of using a shared constant.
    // CHANGES (2025-11-24): AutoSequence.move(...) can now twist to a caller-selected heading relative
    //                        to the move start before finishing the step.
    // CHANGES (2025-11-27): AutoSequence.move(...) now steers toward the twist target during the
    //                        translation instead of turning afterward.
    // CHANGES (2025-11-26): Routed FTC Dashboard telemetry through DecodeFieldDrawing each init/run
    //                        loop with corrected artifact rows and launch triangles so Auto motion
    //                        renders on the Dashboard alongside phone telemetry.
    // CHANGES (2025-11-25): Added odometry fusion + reusable move helpers for field-aware autos and
    //                        exposed start-pose seeding so AprilTag corrections blend cleanly; aligned
    //                        changelog with the 2025-11-25 release.
    // CHANGES (2025-11-25): Blend goal-tag poses using tuned tag locations, seed odometry from
    //                        visible tags during INIT, and persist the final auto pose for TeleOp
    //                        to reuse at startup.
    // CHANGES (2025-11-25): Added AutoSequence.fireContinuous(...) for timed continuous-feed volleys
    //                        that hold the gate open while sustaining launcher RPM.
    // CHANGES (2025-11-25): Corrected AutoSequence.fireContinuous(...) to call the BaseAuto helper
    //                        with the proper scope and parameter order so continuous feeds compile and run.
    // CHANGES (2025-12-09): FTC Dashboard telemetry now mirrors only the Driver Station lines while
    //                        keeping DecodeFieldDrawing overlays for pose context.
    // CHANGES (2025-12-11): Wired BaseAuto to select the active VisionTargetProvider (Limelight default,
    //                        webcam legacy fallback) and route heading/range gating through the provider
    //                        while leaving odometry pose fusion unchanged for this phase.
    // CHANGES (2025-12-11): AutoSequence.visionMode(...) now no-ops when Limelight vision is selected so
    //                        profile swaps remain a webcam-only operation.
    // CHANGES (2025-12-11): Recentered odometry to the field-center frame (human wall = −72" Y), seeded
    //                        default start poses accordingly, and enabled Limelight-only XY fusion with
    //                        MT2 preference, reacquire-friendly outlier gates, motion gating, and
    //                        clamped correction steps (IMU-only heading).
    // CHANGES (2025-12-16): Limelight-backed autos now assert distinct obelisk vs. goal aim pipelines,
    //                        gate aiming strictly to alliance goal tags, and surface pipeline/goal/obelisk
    //                        telemetry so AUTO no longer depends on TeleOp to prep Limelight state.
    // CHANGES (2025-12-16): Added goal-visibility hysteresis (acquire/loss frames + hold window) with
    //                        smoothed heading telemetry so AUTO scans no longer bounce when Limelight
    //                        misses a single frame, and exposed raw/smoothed visibility + tx/loss counts
    //                        in AUTO telemetry.
    // CHANGES (2025-12-16): AutoSequence.move(...) now treats the heading argument as relative to the
    //                        robot's current facing so paths remain robot-centric regardless of start
    //                        orientation; telemetry surfaces both the relative request and resolved
    //                        absolute heading.
    // CHANGES (2025-12-16): Launcher readiness/aim loops now exit immediately upon success while still
    //                        honoring per-step timeouts as fallbacks so AutoSequence steps advance as
    //                        soon as their goals are satisfied.
    // CHANGES (2025-12-19): AutoSequence warm-up/heading-capture steps run without forcing a drive stop,
    //                        readyToLaunch now records whether a tag lock was observed so later steps that
    //                        require lock (e.g., fireContinuous) no longer skip incorrectly, and
    //                        continuous-fire telemetry stays live while AutoSpeed maintains RPM.
    // CHANGES (2025-12-28): Added INIT Limelight pipeline auto-selection with tunable profiles,
    //                        AprilTag precedence scoring, and telemetry severity rules.
    // CHANGES (2025-12-30): Aligned odometry start pose with IMU heading offsets, made init vision
    //                        seeding opt-in, refreshed init telemetry with seed details, and ensured
    //                        dashboard poses use fresh odometry updates each loop.
    // CHANGES (2025-12-31): Ensured Auto saves the final fused pose on stop/cleanup, removed
    //                        double-update dashboard paths, and aligned heading offset telemetry
    // CHANGES (2025-12-31): Allowed FeedStop to open while continuous-fire waits on RPM readiness,
    //                        gating the feed motor instead of the gate motion.
    //                        with the IMU seed definition.
    // CHANGES (2026-01-09): Added an autonomous match timer with per-OpMode endgame reserves,
    //                        MAIN/ENDGAME AutoSequence phases, and main-phase early-exit guards
    //                        so endgame retreat moves can start on time.
    // CHANGES (2026-01-09): Allowed ENDGAME steps to run immediately once MAIN steps finish,
    //                        without waiting for the endgame reserve timer.
    // CHANGES (2026-01-09): Added auto timer and MAIN/ENDGAME status telemetry near the
    //                        top of the Auto status block for clearer runtime context.

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

    /** Override in derived autos to reserve time for endgame actions (per-OpMode constant). */
    protected long endgameReserveMs() { return 0L; }

    // Core subsystems shared by all autos.
    protected Drivebase drive;           // Field-centric mecanum drive helper
    protected VisionAprilTag vision;     // AprilTag pipeline (goal + obelisk)
    protected Limelight3A limelight;     // Limelight device for heading/distance + XY fusion
    protected VisionTargetProvider visionTargetProvider; // Unified heading/range provider (Limelight default)
    protected LimelightPipelineAutoSelector limelightAutoSelector; // INIT Limelight pipeline auto-selector
    protected Launcher launcher;         // Flywheel subsystem for scoring
    protected Feed feed;                 // Artifact feed motor controller
    protected Intake intake;             // Intake roller subsystem
    protected FiringController firingController;

    // Controllers supporting aiming and RPM automation.
    protected final TagAimController aim = new TagAimController();
    protected final LauncherAutoSpeedController autoCtrl = new LauncherAutoSpeedController();
    protected Odometry odometry;                     // Fused drive + IMU + AprilTag pose
    protected FieldPose startPose = new FieldPose(0.0, OdometryConfig.HUMAN_WALL_Y, 0.0); // Staging pose seeded by derived autos
    private boolean visionSeededStart = false;
    private FieldPose odometrySeedPose = null;
    private double seedImuYawDeg = 0.0;
    private double seedHeadingOffsetDeg = 0.0;
    private double seedHeadingDeg = 0.0;
    private String visionSeedReason = "startPose";
    private FieldPose lastDashboardPose = null;
    protected FtcDashboard dashboard;               // Shared FTC Dashboard instance

    // Local defaults if config fails (protects against missing SharedRobotTuning definitions).
    private static final double DEF_LOCK_TOL_DEG   = 1.0;
    private static final double DEF_TURN_CAP       = 0.35;
    private static final double DEF_DRIVE_CAP      = 0.50;
    private static final double DEF_RPM_TOL        = 50.0;
    private static final double DEF_AUTO_SEED_RPM  = 2500.0;
    private static final long   DEF_RPM_SETTLE_MS  = 150L;
    private static final long   DEFAULT_BETWEEN_MS = 3000; // Default between-shot wait used when callers pass ≤ 0
    private static final long   AUTO_DURATION_MS   = 30000L;
    private static final double M_TO_IN            = 39.37007874015748;

    private String autoOpModeName;
    private Integer lastReportedPipelineIndex = null;
    private boolean lastReadyHadLock = false;
    private long autoStartMs = -1L;

    private double lockTolDeg() {
        double fallback = DEF_LOCK_TOL_DEG;
        try { fallback = SharedRobotTuning.LOCK_TOLERANCE_DEG; } catch (Throwable ignored) {}

        if (vision != null) {
            try {
                VisionTuning.Mode mode = vision.getActiveMode();
                if (mode == VisionTuning.Mode.P480) {
                    try { return SharedRobotTuning.LOCK_TOLERANCE_DEG_P480; }
                    catch (Throwable ignored) { return fallback; }
                } else if (mode == VisionTuning.Mode.P720) {
                    try { return SharedRobotTuning.LOCK_TOLERANCE_DEG_P720; }
                    catch (Throwable ignored) { return fallback; }
                }
            } catch (Throwable ignored) {
                // fall through to fallback when vision profile unavailable
            }
        }

        return fallback;
    }
    private double turnTwistCap() { try { return SharedRobotTuning.TURN_TWIST_CAP;     } catch (Throwable t){ return DEF_TURN_CAP;     } }
    private double driveCap()     { try { return SharedRobotTuning.DRIVE_MAX_POWER;    } catch (Throwable t){ return DEF_DRIVE_CAP;    } }
    private double rpmTol()       { try { return SharedRobotTuning.RPM_TOLERANCE;      } catch (Throwable t){ return DEF_RPM_TOL;      } }
    private double autoSeedRpm() {
        try { return SharedRobotTuning.INITIAL_AUTO_DEFAULT_SPEED; }
        catch (Throwable t) { return DEF_AUTO_SEED_RPM; }
    }

    private long rpmSettleMs() {
        try { return SharedRobotTuning.RPM_READY_SETTLE_MS; }
        catch (Throwable t) { return DEF_RPM_SETTLE_MS; }
    }

    private boolean isAimReadyForFire() {
        VisionTargetProvider provider = visionTargetProvider;
        if (provider == null || !provider.isGoalVisibleSmoothed()) {
            return false;
        }
        Double heading = provider.getSmoothedHeadingErrorDeg();
        if (heading == null || Double.isNaN(heading)) {
            return false;
        }
        return Math.abs(heading) <= lockTolDeg();
    }

    private void updateLauncherReadyLatch(long nowMs) {
        if (launcher == null) {
            return;
        }
        launcher.updateReadyLatch(nowMs, launcher.targetRpm);
    }

    /** Elapsed autonomous time (ms) since START. */
    protected final long autoElapsedMs() {
        if (autoStartMs < 0L) {
            return 0L;
        }
        return Math.max(0L, System.currentTimeMillis() - autoStartMs);
    }

    /** Remaining autonomous time (ms), clamped to ≥ 0. */
    protected final long autoRemainingMs() {
        return Math.max(0L, AUTO_DURATION_MS - autoElapsedMs());
    }

    /** Remaining MAIN-phase time (ms) after reserving endgame, clamped to ≥ 0. */
    protected final long mainRemainingMs() {
        return Math.max(0L, autoRemainingMs() - Math.max(0L, endgameReserveMs()));
    }

    /** True when MAIN phase is over and ENDGAME steps should begin. */
    protected final boolean mainPhaseOver() {
        return mainRemainingMs() <= 0L;
    }


    @Override
    public void runOpMode() throws InterruptedException {
        autoOpModeName = resolveAutoName();

        // Create drivetrain with IMU + encoder helpers for field-aligned movement.
        drive = new Drivebase(this);
        limelight = null;
        // Initialize AprilTag vision (legacy webcam path) and the unified target provider.
        try {
            vision = new VisionAprilTag();
            vision.init(hardwareMap, "Webcam 1");
            try { vision.setRangeScale(VisionTuning.RANGE_SCALE); } catch (Throwable ignored) {}
        } catch (Exception ex) { vision = null; }

        visionTargetProvider = null;
        limelightAutoSelector = null;
        try {
            VisionConfig.VisionSource source = VisionConfig.VISION_SOURCE;
            if (source == VisionConfig.VisionSource.WEBCAM_LEGACY) {
                if (vision != null) {
                    try { vision.setGoalVisibilityTargetId(allianceGoalTagId()); } catch (Throwable ignored) {}
                }
                visionTargetProvider = new WebcamLegacyTargetProvider(vision, this::alliance);
            } else {
                try {
                    limelight = hardwareMap.get(Limelight3A.class, "limelight");
                    applyLimelightPollRate(limelight);
                } catch (Exception ex) { limelight = null; }
                visionTargetProvider = new LimelightTargetProvider(limelight, this::alliance);
                limelightAutoSelector = new LimelightPipelineAutoSelector(
                        limelight,
                        (visionTargetProvider instanceof LimelightTargetProvider)
                                ? (LimelightTargetProvider) visionTargetProvider
                                : null,
                        this::alliance);
                if (limelight != null) {
                    if (limelightAutoSelector == null || !limelightAutoSelector.isEnabled()) {
                        applyLimelightPipeline(limelight);
                    }
                    try { limelight.start(); } catch (Throwable ignored) {}
                }
            }
        } catch (Exception ex) {
            visionTargetProvider = null;
        }
        aim.setProvider(visionTargetProvider);
        odometry = new Odometry(drive, limelight);
        dashboard = FtcDashboard.getInstance();
        launcher = new Launcher(hardwareMap);   // Flywheel pair
        feed     = new Feed(hardwareMap);       // Indexer wheel
        intake   = new Intake(hardwareMap);     // Floor intake
        firingController = new FiringController(feed, intake);

        // Guarantee INIT remains motionless until START.
        drive.safeInit();
        launcher.safeInit();
        feed.safeInit();
        intake.safeInit();
        if (firingController != null) {
            firingController.setIntakeDesiredState(intake.isOn());
        }
        feed.initFeedStop(hardwareMap, telemetry);
        seedOdometryFromPose(startPose, "startPose", false);

        try { AutoRpmConfig.apply(autoCtrl); } catch (Throwable ignored) {} // Sync AutoSpeed curve
        ObeliskSignal.clear(); // Reset Obelisk latch before looking for motifs

        while (!isStarted() && !isStopRequested()) {
            if (limelightAutoSelector != null && limelightAutoSelector.isEnabled()) {
                limelightAutoSelector.update();
            } else {
                ensureLimelightObeliskMode();
            }
            FieldPose initPose = updateOdometryPose();
            maybeSeedStartPoseFromVision();
            updateStatusWithPose("INIT", false, initPose);
            onPreStartLoop();
            telemetry.update();
            sleep(20);
        }
        if (isStopRequested()) {
            FieldPose finalPose = updateOdometryPose();
            saveFinalPoseForTeleOp(finalPose);
            stopVisionIfAny();
            return;
        }
        autoStartMs = System.currentTimeMillis();
        if (limelightAutoSelector != null) {
            limelightAutoSelector.notifyOpModeStarted();
        }
        feed.startFeedStopAfterStart();
        feed.setIdleHoldActive(true); // Allow idle counter-rotation only after START
        intake.set(true);             // Mirror TeleOp default: intake runs once the match starts
        if (firingController != null) {
            firingController.setIntakeDesiredState(true);
        }
        if (vision != null) vision.setObeliskAutoLatchEnabled(true); // Capture motifs during movement

        try { runSequence(); }
        finally {
            FieldPose finalPose = updateOdometryPose();
            saveFinalPoseForTeleOp(finalPose);
            stopAll();
            stopVisionIfAny();
            updateStatusWithPose("COMPLETE", false, finalPose);
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
     * @param timeoutMs          Maximum time to spend sweeping before abandoning the scan and continuing.
     */
    protected final boolean turnToGoalTag(String phase,
                                           ScanDirection direction,
                                           double turnSpeedFraction,
                                           double primarySweepDeg,
                                           Double oppositeSweepDeg,
                                           long timeoutMs) {
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

        LockWindow lockWindow = computeLockWindow(false, tol);
        aim.setDeadbandWindow(lockWindow.minDeg, lockWindow.maxDeg);

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

        ensureLimelightGoalAimMode();
        SweepState state = SweepState.PRIMARY_OUT;
        long startMs = System.currentTimeMillis();

        while (opModeIsActive()) {
            if (mainPhaseOver()) {
                drive.stopAll();
                return false;
            }
            FieldPose loopPose = updateOdometryPose();
            long now = System.currentTimeMillis();
            if (timeoutMs > 0 && (now - startMs) >= timeoutMs) {
                drive.stopAll();
                updateStatusWithPose(label + " – timeout", false, loopPose);
                telemetry.addData("Bearing (deg)", "---");
                telemetry.addData("Turn speed (|twist|)", twist);
                telemetry.addData("Sweep offsets (deg)", sweepSummary);
                telemetry.addData("Time remaining (ms)", 0);
                telemetry.update();
                return false;
            }

            updateIntakeFlowForAuto();
            ensureLimelightGoalAimMode();
            VisionTargetProvider provider = visionTargetProvider;
            double bearing = Double.NaN;
            boolean lockedNow = false;

            boolean smoothedGoal = provider != null && provider.isGoalVisibleSmoothed();
            Double smoothedHeading = (provider != null) ? provider.getSmoothedHeadingErrorDeg() : null;

            if (smoothedGoal && smoothedHeading != null) {
                Double distanceIn = distanceInchesFromProvider(provider);
                lockWindow = computeLockWindow(isLongShot(distanceIn), tol);
                aim.setDeadbandWindow(lockWindow.minDeg, lockWindow.maxDeg);
                double err = smoothedHeading;
                bearing = err;
                double cmdRaw = aim.turnPower();
                double cmd = clamp(TagAimController.applyDriveTwistSign(cmdRaw), -cap, +cap);
                drive.drive(0, 0, cmd);
                lockedNow = lockWindow.contains(err);
                if (lockedNow) {
                    drive.stopAll();
                    updateStatusWithPose(label + " – lock", true, loopPose);
                    telemetry.addData("Bearing (deg)", err);
                    telemetry.update();
                    return true;
                }
            } else {
                lockWindow = computeLockWindow(false, tol);
                aim.setDeadbandWindow(lockWindow.minDeg, lockWindow.maxDeg);
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

            updateStatusWithPose(label, lockedNow, loopPose);
            telemetry.addData("Bearing (deg)", bearing);
            telemetry.addData("Turn speed (|twist|)", twist);
            telemetry.addData("Sweep offsets (deg)", sweepSummary);
            if (timeoutMs > 0) {
                telemetry.addData("Time remaining (ms)", Math.max(0, timeoutMs - (now - startMs)));
            }
            telemetry.update();
            idle();
        }
        drive.stopAll();
        FieldPose cancelPose = updateOdometryPose();
        updateStatusWithPose(label + " – cancelled", false, cancelPose);
        telemetry.update();
        return false;
    }

    // ---------- Spin to at-speed ----------
    /**
     * Enable AutoSpeed, feed AprilTag distance into the curve, and wait for launcher RPM
     * to enter the tuned tolerance window. Returns true when ready before timeout.
     */
    protected final boolean readyLauncherUntilReady(long timeoutMs) {
        return readyLauncherUntilReady(timeoutMs, "Ready launcher");
    }

    protected final boolean readyLauncherUntilReady(long timeoutMs, String phase) {
        drive.stopAll();
        autoCtrl.setAutoEnabled(true);
        try { AutoRpmConfig.apply(autoCtrl); } catch (Throwable ignored) {}

        final long settleMs = rpmSettleMs();
        final double tolerance = rpmTol();
        final double fallbackRpm = autoSeedRpm();
        try { autoCtrl.setDefaultRpm(fallbackRpm); } catch (Throwable ignored) {}
        lastReadyHadLock = false;

        boolean hadLock = false;
        long start = System.currentTimeMillis();
        long deadline = (timeoutMs > 0) ? start + timeoutMs : Long.MAX_VALUE;
        long settleStart = -1L;

        while (opModeIsActive()) {
            updateIntakeFlowForAuto();
            FieldPose loopPose = updateOdometryPose();
            long now = System.currentTimeMillis();
            if (mainPhaseOver()) {
                updateStatusWithPose(phase + " – endgame reserve", false, loopPose);
                telemetry.update();
                lastReadyHadLock = false;
                return false;
            }
            if (now >= deadline) {
                updateStatusWithPose(phase + " – timeout", false, loopPose);
                telemetry.addData("Phase", phase);
                telemetry.addData("Distance (in)", "---");
                telemetry.addData("Target RPM", launcher.targetRpm);
                telemetry.addData("Current RPM", launcher.getCurrentRpm());
                telemetry.addData("Tolerance", tolerance);
                telemetry.addData("Within band", false);
                telemetry.addData("Time remaining (ms)", 0);
                telemetry.update();
                lastReadyHadLock = false;
                return false;
            }

            Double distanceIn = distanceInchesFromProvider(visionTargetProvider);
            if (distanceIn != null) { hadLock = true; }

            double targetRpm;
            if (distanceIn != null) {
                targetRpm = autoCtrl.updateWithVision(distanceIn);
            } else if (hadLock) {
                targetRpm = autoCtrl.updateWithVision(null);
            } else {
                targetRpm = fallbackRpm;
            }

            launcher.setTargetRpm(targetRpm);
            updateLauncherReadyLatch(now);

            double currentRpm = launcher.getCurrentRpm();
            double error = Math.abs(currentRpm - launcher.targetRpm);
            boolean withinBand = error <= tolerance;
            String distanceText = (distanceIn == null) ? "---" : String.format(Locale.US, "%.1f", distanceIn);

            if (withinBand && hadLock) {
                if (settleStart < 0) settleStart = now;
                if ((now - settleStart) >= settleMs) {
                    updateStatusWithPose(phase + " – ready", true, loopPose);
                    telemetry.addData("Phase", phase);
                    telemetry.addData("Distance (in)", distanceText);
                    telemetry.addData("Target RPM", launcher.targetRpm);
                    telemetry.addData("Current RPM", currentRpm);
                    telemetry.addData("Tolerance", tolerance);
                    telemetry.addData("Within band", withinBand);
                    telemetry.addData("Time remaining (ms)", Math.max(0, timeoutMs - (now - start)));
                    telemetry.update();
                    lastReadyHadLock = true;
                    return true;
                }
            } else {
                settleStart = -1L;
            }

            updateStatusWithPose(phase, hadLock && distanceIn != null, loopPose);
            telemetry.addData("Phase", phase);
            telemetry.addData("Distance (in)", distanceText);
            telemetry.addData("Target RPM", launcher.targetRpm);
            telemetry.addData("Current RPM", currentRpm);
            telemetry.addData("Tolerance", tolerance);
            telemetry.addData("Within band", withinBand);
            telemetry.addData("Time remaining (ms)", Math.max(0, timeoutMs - (now - start)));
            telemetry.update();

            idle();
        }

        updateStatusWithPose(phase + " – timeout", false, updateOdometryPose());
        telemetry.addData("Phase", phase);
        telemetry.addData("Distance (in)", "---");
        telemetry.addData("Target RPM", launcher.targetRpm);
        telemetry.addData("Current RPM", launcher.getCurrentRpm());
        telemetry.addData("Tolerance", tolerance);
        telemetry.addData("Within band", false);
        telemetry.addData("Time remaining (ms)", 0);
        telemetry.update();
        lastReadyHadLock = false;
        return false;
    }

    // ---------- Strictly gated shooting ----------
    /**
     * Fire count artifacts with strict gating—requires tag lock for each shot and enforces
     * both RPM readiness and the caller-supplied between-shot delay.
     */
    protected final void fireN(int count) throws InterruptedException {
        fireN(count, true, true, DEFAULT_BETWEEN_MS);
    }

    protected final void fireN(int count, boolean requireLock) throws InterruptedException {
        fireN(count, requireLock, true, DEFAULT_BETWEEN_MS);
    }

    protected final void fireN(int count, boolean requireLock, boolean requireLauncherAtSpeed) throws InterruptedException {
        fireN(count, requireLock, requireLauncherAtSpeed, DEFAULT_BETWEEN_MS);
    }

    protected final void fireN(int count, boolean requireLock, boolean requireLauncherAtSpeed, long betweenShotsMs)
            throws InterruptedException {
        autoCtrl.setAutoEnabled(true);
        boolean resumeIntakeAfterFire = intake.isOn();
        for (int i = 0; i < count && opModeIsActive(); i++) {
            if (mainPhaseOver()) {
                break;
            }
            final String shotPhase = String.format("Volley %d/%d", i + 1, count);
            boolean lockedForShot = !requireLock;
            if (requireLock) {
                if (mainPhaseOver()) {
                    break;
                }
                lockedForShot = requireLockOrTimeOut(1200, shotPhase + " – acquire lock");
                if (!lockedForShot) {
                    updateStatusWithPose("Hold position", false, updateOdometryPose());
                    telemetry.addLine("⚠️ No tag lock — skipping shot " + (i + 1));
                    telemetry.update();
                    continue; // do not free-fire when lock required
                }
            }
            if (mainPhaseOver()) {
                break;
            }

            double holdTarget = autoCtrl.hold();
            if (holdTarget <= 0) {
                holdTarget = launcher.targetRpm;
                if (holdTarget <= 0) {
                    holdTarget = autoSeedRpm();
                }
            }
            launcher.setTargetRpm(holdTarget);
            updateLauncherReadyLatch(System.currentTimeMillis());

            boolean sprayLike = !requireLock && !requireLauncherAtSpeed;
            SharedRobotTuning.HoldFireForRpmMode rpmGateMode = requireLauncherAtSpeed
                    ? SharedRobotTuning.HoldFireForRpmMode.ALL
                    : SharedRobotTuning.HoldFireForRpmMode.OFF;

            if (firingController != null) {
                firingController.setIntakeDesiredState(intake.isOn());
                firingController.requestFire(
                        System.currentTimeMillis(),
                        false,
                        sprayLike,
                        launcher != null && launcher.isReadyLatched(),
                        intake.isOn(),
                        rpmGateMode,
                        true,
                        TeleOpDriverDefaults.FIRING_AUTO_AIM_TIME_THRESHOLD_MS);
            }

            while (opModeIsActive() && !mainPhaseOver() && firingController != null && !firingController.isIdle()) {
                long now = System.currentTimeMillis();
                firingController.setIntakeDesiredState(intake.isOn());
                firingController.setContinuousRequested(false);
                firingController.setSprayLike(sprayLike);
                updateLauncherReadyLatch(now);
                firingController.update(now,
                        isAimReadyForFire(),
                        launcher.targetRpm,
                        launcher.getLeftRpm(),
                        launcher.getRightRpm(),
                        intake.isOn());

                feed.update();
                updateIntakeFlowForAuto();

                FieldPose loopPose = updateOdometryPose();
                updateStatusWithPose(shotPhase, lockedForShot || !requireLock, loopPose);
                telemetry.addData("Target RPM", launcher.targetRpm);
                telemetry.addData("Current RPM", launcher.getCurrentRpm());
                telemetry.update();
                idle();
            }
            if (mainPhaseOver()) {
                break;
            }

            // Reassert the AutoSpeed target so the flywheels stay at commanded RPM during recovery.
            double recoverTarget = autoCtrl.hold();
            if (recoverTarget <= 0) {
                recoverTarget = holdTarget;
            }
            launcher.setTargetRpm(recoverTarget);

            long delay = (betweenShotsMs > 0) ? betweenShotsMs : DEFAULT_BETWEEN_MS;
            if (mainPhaseOver()) {
                break;
            }
            sleep((int)delay);
            feed.update();
            updateIntakeFlowForAuto();
            if (resumeIntakeAfterFire) {
                intake.set(true);
                if (firingController != null) {
                    firingController.setIntakeDesiredState(true);
                }
            }
            drive.stopAll();
            updateStatusWithPose("Stabilize after volley", lockedForShot || !requireLock, updateOdometryPose());
            telemetry.update();
        }
    }

    /** Run a continuous feed stream for the requested duration, optionally requiring a tag lock first. */
    protected final void fireContinuous(long durationMs, boolean requireLock, boolean requireLauncherAtSpeed, String label)
            throws InterruptedException {
        long runMs = Math.max(0L, durationMs);
        if (runMs <= 0L) {
            return;
        }
        if (mainPhaseOver()) {
            return;
        }

        boolean resumeIntakeAfterFire = intake.isOn();
        boolean lockedForRun = !requireLock;
        if (requireLock) {
            if (mainPhaseOver()) {
                return;
            }
            lockedForRun = lastReadyHadLock || requireLockOrTimeOut(1200, label + " – acquire lock");
            if (!lockedForRun) {
                updateStatusWithPose(label + " – skipped (no lock)", false, updateOdometryPose());
                telemetry.addLine("⚠️ Continuous fire skipped because tag lock was not achieved");
                telemetry.update();
                return;
            }
        }

        autoCtrl.setAutoEnabled(true);
        double holdTarget = autoCtrl.hold();
        if (holdTarget <= 0) {
            holdTarget = launcher.targetRpm;
            if (holdTarget <= 0) {
                holdTarget = autoSeedRpm();
            }
        }
        launcher.setTargetRpm(holdTarget);
        updateLauncherReadyLatch(System.currentTimeMillis());

        boolean sprayLike = !requireLock && !requireLauncherAtSpeed;
        SharedRobotTuning.HoldFireForRpmMode rpmGateMode = requireLauncherAtSpeed
                ? SharedRobotTuning.HoldFireForRpmMode.ALL
                : SharedRobotTuning.HoldFireForRpmMode.OFF;

        long start = System.currentTimeMillis();
        long endMs = start + runMs;
        while (opModeIsActive() && !mainPhaseOver()) {
            long now = System.currentTimeMillis();
            boolean continueRequested = now < endMs;

            double sustainTarget = autoCtrl.hold();
            if (sustainTarget > 0) {
                launcher.setTargetRpm(sustainTarget);
            }

            if (firingController != null) {
                firingController.setIntakeDesiredState(intake.isOn());
                if (continueRequested && firingController.isIdle()) {
                    firingController.requestFire(
                            now,
                            true,
                            sprayLike,
                            launcher != null && launcher.isReadyLatched(),
                            intake.isOn(),
                            rpmGateMode,
                            true,
                            TeleOpDriverDefaults.FIRING_AUTO_AIM_TIME_THRESHOLD_MS);
                }
                firingController.setContinuousRequested(continueRequested);
                firingController.setSprayLike(sprayLike && continueRequested);
                updateLauncherReadyLatch(now);
                firingController.update(now,
                        isAimReadyForFire(),
                        launcher.targetRpm,
                        launcher.getLeftRpm(),
                        launcher.getRightRpm(),
                        intake.isOn());
            }

            feed.update();
            updateIntakeFlowForAuto();

            updateStatusWithPose(label, lockedForRun || !requireLock, updateOdometryPose());
            telemetry.addData("Target RPM", launcher.targetRpm);
            telemetry.addData("Current RPM", launcher.getCurrentRpm());
            telemetry.addData("Time remaining (ms)", Math.max(0L, endMs - now));
            telemetry.update();

            if (!continueRequested && (firingController == null || firingController.isIdle())) {
                break;
            }
            idle();
        }

        drive.stopAll();
        if (resumeIntakeAfterFire) {
            intake.set(true);
            if (firingController != null) {
                firingController.setIntakeDesiredState(true);
            }
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
        final double tol = lockTolDeg();
        final double cap = turnTwistCap();
        final String label = (phase == null || phase.isEmpty()) ? "Acquire lock" : phase;
        ScanDirection resolved = (direction == null) ? ScanDirection.CW : direction;
        boolean cwFirst = resolved.isClockwise();
        double scanSign = cwFirst ? -1.0 : +1.0;
        long start = System.currentTimeMillis();
        long lastFlip = start;

        while (opModeIsActive() && !mainPhaseOver() && (System.currentTimeMillis() - start) < guardMs) {
            updateIntakeFlowForAuto();
            VisionTargetProvider provider = visionTargetProvider;
            Double smoothedHeading = (provider != null) ? provider.getSmoothedHeadingErrorDeg() : null;
            if (provider != null && provider.isGoalVisibleSmoothed() && smoothedHeading != null) {
                double err = smoothedHeading;
                boolean locked = Math.abs(err) <= tol;
                if (locked) {
                    drive.stopAll();
                    updateStatusWithPose(label, true, updateOdometryPose());
                    telemetry.addData("Bearing (deg)", err);
                    telemetry.update();
                    return true;
                }
                double cmdRaw = aim.turnPower();
                double cmd = clamp(TagAimController.applyDriveTwistSign(cmdRaw), -cap, +cap);
                drive.drive(0, 0, cmd * 0.6);
                updateStatusWithPose(label, false, updateOdometryPose());
                telemetry.addData("Bearing (deg)", err);
            } else {
                long now = System.currentTimeMillis();
                if (now - lastFlip > 600) { scanSign *= -1.0; lastFlip = now; }
                drive.drive(0, 0, scanSign * 0.25 * cap);
                updateStatusWithPose(label, false, updateOdometryPose());
                telemetry.addData("Bearing (deg)", Double.NaN);
            }
            telemetry.update();
            idle();
        }
        drive.stopAll();
        updateStatusWithPose(label + " – timeout", false, updateOdometryPose());
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

    /** Seed odometry before the match begins so INIT telemetry reflects the staging pose. */
    protected final void setStartingPose(double x, double y, double headingDeg) {
        startPose = new FieldPose(x, y, headingDeg);
        if (odometry != null) {
            seedOdometryFromPose(startPose, "startPose", false);
        }
    }

    /** Refresh odometry and return the latest fused pose. */
    protected final FieldPose updateOdometryPose() {
        if (odometry == null) return new FieldPose();
        return odometry.update();
    }

    private void maybeSeedStartPoseFromVision() {
        if (visionSeededStart || odometry == null) return;
        if (!VisionConfig.LimelightFusion.INIT_ALLOW_VISION_SEED) {
            visionSeedReason = "disabled";
            return;
        }
        FieldPose guess = odometry.getLastVisionPose();
        if (guess == null && VisionConfig.VISION_SOURCE == VisionConfig.VisionSource.LIMELIGHT) {
            odometry.update();
            guess = odometry.getLastVisionPose();
        }
        if (guess != null) {
            startPose = guess;
            seedOdometryFromPose(guess, "vision", true);
            visionSeededStart = true;
        } else {
            visionSeedReason = "no_tag";
        }
    }

    /** Move to a field position using odometry, ending at the desired heading. */
    protected final void moveToPosition(String label, double targetX, double targetY, double speedCap, double finalHeadingDeg) {
        FieldPose cur = updateOdometryPose();
        double dx = targetX - cur.x;
        double dy = targetY - cur.y;
        double distance = Math.hypot(dx, dy);
        if (distance < 1e-3) return;
        double headingDeg = Math.toDegrees(Math.atan2(dx, dy));
        drive.moveWithTwist(distance, headingDeg, finalHeadingDeg, clampTranslationSpeed(speedCap), clampTurnSpeed(speedCap));
        updateStatusWithPose(label, true, updateOdometryPose());
        telemetry.update();
        updateOdometryPose();
    }

    /** Alliance-aware helper to align the intake with the correct artifact row and collect while reversing. */
    protected final void intakeFieldArtifacts(double approachSpeed, double reverseSpeed) {
        ObeliskSignal.Order order = ObeliskSignal.get();
        double rowStartX = (alliance() == Alliance.RED)
                ? OdometryConfig.RED_ARTIFACT_ROW_START_X
                : OdometryConfig.BLUE_ARTIFACT_ROW_START_X;
        double rowY;
        switch (order) {
            case GPP: rowY = OdometryConfig.GPP_ROW_Y; break;
            case PPG: rowY = OdometryConfig.PPG_ROW_Y; break;
            case PGP:
            default: rowY = OdometryConfig.PGP_ROW_Y; break;
        }
        double alignX = rowStartX + OdometryConfig.ARTIFACT_SPACING_X - OdometryConfig.INTAKE_OFFSET_X; // center of the row trio
        moveToPosition("Align artifacts", alignX, rowY, approachSpeed, 0.0);
        intake.set(true);
        drive.moveWithTwist(-OdometryConfig.ARTIFACT_SPACING_X * 3.0, 0.0, 0.0,
                clampTranslationSpeed(reverseSpeed), clampTurnSpeed(reverseSpeed));
        updateOdometryPose();
    }

    /** Stop all active subsystems (safety catch-all). */
    protected final void stopAll() {
        try { drive.applyBrakeHold(); } catch (Throwable ignored) {}
        try { launcher.applyBrakeHold(); } catch (Throwable ignored) {}
        try { feed.setIdleHoldActive(false); feed.applyBrakeHold(); } catch (Throwable ignored) {}
        try { intake.applyBrakeHold(); } catch (Throwable ignored) {}
        try { autoCtrl.setAutoEnabled(false); } catch (Throwable ignored) {}
        if (firingController != null) {
            firingController.reset();
        }
    }
    private void applyLimelightPipeline(Limelight3A ll) {
        if (ll == null) return;
        try {
            ll.pipelineSwitch(VisionConfig.LimelightFusion.OBELISK_PIPELINE_INDEX);
            return;
        } catch (Throwable ignored) { }

        try {
            ll.getClass().getMethod("setPipelineIndex", int.class)
                    .invoke(ll, VisionConfig.LimelightFusion.OBELISK_PIPELINE_INDEX);
        } catch (Throwable ignored) { }
    }

    private void applyLimelightPollRate(Limelight3A ll) {
        if (ll == null) return;
        try {
            ll.setPollRateHz(VisionConfig.LimelightFusion.POLL_HZ);
        } catch (Throwable ignored) { }
    }

    private void ensureLimelightObeliskMode() {
        if (limelightAutoSelector != null && limelightAutoSelector.isLocked()) {
            return;
        }
        if (visionTargetProvider == null) return;
        Integer before = visionTargetProvider.getActivePipelineIndex();
        visionTargetProvider.ensureObeliskObservationMode();
        maybeReportPipelineSwitch(before, visionTargetProvider.getActivePipelineIndex(),
                "AUTO: Limelight -> Obelisk observation mode");
    }

    private void ensureLimelightGoalAimMode() {
        if (limelightAutoSelector != null && limelightAutoSelector.isLocked()) {
            return;
        }
        if (visionTargetProvider == null) return;
        Integer before = visionTargetProvider.getActivePipelineIndex();
        visionTargetProvider.ensureGoalAimMode(alliance());
        maybeReportPipelineSwitch(before, visionTargetProvider.getActivePipelineIndex(),
                String.format(Locale.US, "AUTO: Limelight -> Goal aim mode (%s)", alliance()));
    }

    private void maybeReportPipelineSwitch(Integer before, Integer after, String message) {
        if (after == null) return;
        if (!Objects.equals(after, before) || !Objects.equals(after, lastReportedPipelineIndex)) {
            telemetry.addLine(String.format(Locale.US, "%s", message + " (pipeline=" + after + ")"));
            lastReportedPipelineIndex = after;
        }
    }
    /** Shutdown the vision portal safely if it was created. */
    protected final void stopVisionIfAny() {
        try { if (vision != null) { vision.setObeliskAutoLatchEnabled(false); vision.stop(); } } catch (Exception ignored) {}
        try { if (limelight != null) { limelight.stop(); } } catch (Exception ignored) {}
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

    protected final void updateStatusWithPose(String phase, boolean tagLocked, FieldPose poseForDashboard) {
        if (limelightAutoSelector != null && limelightAutoSelector.isEnabled() && !limelightAutoSelector.isLocked()) {
            limelightAutoSelector.update();
        }
        lastDashboardPose = (poseForDashboard != null) ? poseForDashboard : startPose;
        if (feed != null) {
            try { feed.update(); } catch (Throwable ignored) {}
        }
        if (vision != null) {
            try { vision.observeObelisk(); } catch (Throwable ignored) {}
        }
        String statusPhase = (phase == null) ? "" : phase.trim();
        if (statusPhase.isEmpty()) {
            statusPhase = "Sequence";
        }
        List<String> mirroredLines = new ArrayList<>();
        if (limelightAutoSelector != null) {
            String fallbackLine = limelightAutoSelector.getFallbackBannerLine();
            if (fallbackLine != null) {
                telemetry.addLine(fallbackLine);
                mirroredLines.add(fallbackLine);
            }
        }

        telemetry.addData("Phase", statusPhase);
        mirroredLines.add("Phase: " + statusPhase);

        String autoPhaseLabel = mainPhaseOver() ? "ENDGAME" : "MAIN";
        String timerLine = String.format(Locale.US,
                "Elapsed %d / Remaining %d / Main %d",
                autoElapsedMs(),
                autoRemainingMs(),
                mainRemainingMs());
        telemetry.addData("Auto Timer (ms)", timerLine);
        telemetry.addData("Auto Phase", autoPhaseLabel);
        mirroredLines.add("Auto Timer (ms): " + timerLine);
        mirroredLines.add("Auto Phase: " + autoPhaseLabel);

        telemetry.addLine("");
        mirroredLines.add("");

        telemetry.addData("Alliance", alliance());
        mirroredLines.add("Alliance: " + alliance());

        telemetry.addData("Auto", autoOpModeName);
        mirroredLines.add("Auto: " + autoOpModeName);

        String startPoseText = startPoseDescription();
        telemetry.addData("Start Pose", startPoseText);
        mirroredLines.add("Start Pose: " + startPoseText);
        if (!isStarted()) {
            String seedPoseStr = (odometrySeedPose == null)
                    ? "--,--,--"
                    : String.format(Locale.US, "%.1f,%.1f,%.1f", odometrySeedPose.x, odometrySeedPose.y, odometrySeedPose.headingDeg);
            String startPoseStr = String.format(Locale.US, "%.1f,%.1f,%.1f", startPose.x, startPose.y, startPose.headingDeg);
            String seedHeadingStr = String.format(Locale.US, "%.1f", seedHeadingDeg);
            telemetry.addData("Start Pose Seed", startPoseStr);
            telemetry.addData("Odo After Seed", seedPoseStr);
            telemetry.addData("IMU Yaw At Seed", String.format(Locale.US, "%.1f", seedImuYawDeg));
            telemetry.addData("Seed Heading", seedHeadingStr);
            telemetry.addData("Heading Offset", String.format(Locale.US, "%.1f", seedHeadingOffsetDeg));
            telemetry.addData("Vision Seeded", visionSeededStart);
            telemetry.addData("Vision Seed Reason", visionSeedReason);
            mirroredLines.add("Start Pose Seed: " + startPoseStr);
            mirroredLines.add("Odo After Seed: " + seedPoseStr);
            mirroredLines.add("IMU Yaw At Seed: " + String.format(Locale.US, "%.1f", seedImuYawDeg));
            mirroredLines.add("Seed Heading: " + seedHeadingStr);
            mirroredLines.add("Heading Offset: " + String.format(Locale.US, "%.1f", seedHeadingOffsetDeg));
            mirroredLines.add("Vision Seeded: " + visionSeededStart);
            mirroredLines.add("Vision Seed Reason: " + visionSeedReason);
        }
        double imuYawNow = (drive != null)
                ? normDeg(drive.heading() + OdometryConfig.IMU_HEADING_OFFSET_DEG)
                : 0.0;
        double fusedHeadingNow = (poseForDashboard != null) ? poseForDashboard.headingDeg : 0.0;
        telemetry.addData("IMU Yaw Now", String.format(Locale.US, "%.1f", imuYawNow));
        telemetry.addData("Fused Heading Now", String.format(Locale.US, "%.1f", fusedHeadingNow));
        mirroredLines.add("IMU Yaw Now: " + String.format(Locale.US, "%.1f", imuYawNow));
        mirroredLines.add("Fused Heading Now: " + String.format(Locale.US, "%.1f", fusedHeadingNow));

        boolean goalVisibleRaw = visionTargetProvider != null && visionTargetProvider.isGoalVisibleRaw();
        boolean goalVisibleSmoothed = visionTargetProvider != null && visionTargetProvider.isGoalVisibleSmoothed();
        Double txToUseDeg = (visionTargetProvider != null) ? visionTargetProvider.getSmoothedHeadingErrorDeg() : null;
        int goalLostFrames = (visionTargetProvider != null) ? visionTargetProvider.getGoalLostFrames() : 0;
        Integer pipelineIdx = (visionTargetProvider != null) ? visionTargetProvider.getActivePipelineIndex() : null;
        List<Integer> visibleObelisks = (visionTargetProvider != null) ? visionTargetProvider.getVisibleObeliskIds() : new ArrayList<>();
        int latchedObeliskId = (visionTargetProvider != null) ? visionTargetProvider.getLatchedObeliskId() : -1;

        telemetry.addData("LL Pipeline", pipelineIdx == null ? "-" : pipelineIdx);
        mirroredLines.add("LL Pipeline: " + (pipelineIdx == null ? "-" : pipelineIdx));
        telemetry.addData("Goal Tag Visible (raw)", goalVisibleRaw);
        mirroredLines.add("Goal Tag Visible (raw): " + goalVisibleRaw);
        telemetry.addData("Goal Tag Visible (smoothed)", goalVisibleSmoothed);
        mirroredLines.add("Goal Tag Visible (smoothed): " + goalVisibleSmoothed);
        telemetry.addData("Goal Lost Frames", goalLostFrames);
        mirroredLines.add("Goal Lost Frames: " + goalLostFrames);
        telemetry.addData("Goal tx to use (deg)", txToUseDeg == null ? "---" : String.format(Locale.US, "%.1f", txToUseDeg));
        mirroredLines.add("Goal tx to use (deg): " + (txToUseDeg == null ? "---" : String.format(Locale.US, "%.1f", txToUseDeg)));
        telemetry.addData("Obelisk Seen", visibleObelisks.isEmpty() ? "-" : visibleObelisks.toString());
        mirroredLines.add("Obelisk Seen: " + (visibleObelisks.isEmpty() ? "-" : visibleObelisks.toString()));
        telemetry.addData("Obelisk Latched", latchedObeliskId < 0 ? "-" : latchedObeliskId);
        mirroredLines.add("Obelisk Latched: " + (latchedObeliskId < 0 ? "-" : latchedObeliskId));

        Double tagDistanceIn = null;
        Double rawTzM = null;
        Double rawTzIn = null;
        Double fieldIn = null;
        if (visionTargetProvider != null && visionTargetProvider.hasAnyTarget()) {
            double rangeM = visionTargetProvider.getDistanceMeters();
            if (Double.isFinite(rangeM)) {
                tagDistanceIn = rangeM * M_TO_IN;
            }
            if (visionTargetProvider instanceof LimelightTargetProvider) {
                LimelightTargetProvider.DistanceEstimate dist =
                        ((LimelightTargetProvider) visionTargetProvider).getLastDistanceEstimate();
                if (dist != null) {
                    if (dist.targetForwardMeters != null && Double.isFinite(dist.targetForwardMeters)) {
                        rawTzM = dist.targetForwardMeters;
                        rawTzIn = rawTzM * M_TO_IN;
                    }
                    if (dist.fieldDistanceMeters != null && Double.isFinite(dist.fieldDistanceMeters)) {
                        fieldIn = dist.fieldDistanceMeters * M_TO_IN;
                    }
                }
            }
        }

        telemetry.addData("Tag Distance (in)", (tagDistanceIn == null) ? "---" : String.format(Locale.US, "%.1f", tagDistanceIn));
        mirroredLines.add("Tag Distance (in): " + ((tagDistanceIn == null) ? "---" : String.format(Locale.US, "%.1f", tagDistanceIn)));
        telemetry.addData("rawTZ_m", (rawTzM == null) ? "---" : String.format(Locale.US, "%.3f", rawTzM));
        mirroredLines.add("rawTZ_m: " + ((rawTzM == null) ? "---" : String.format(Locale.US, "%.3f", rawTzM)));
        telemetry.addData("rawTZ_in", (rawTzIn == null) ? "---" : String.format(Locale.US, "%.1f", rawTzIn));
        mirroredLines.add("rawTZ_in: " + ((rawTzIn == null) ? "---" : String.format(Locale.US, "%.1f", rawTzIn)));
        telemetry.addData("fieldDistanceIn", (fieldIn == null) ? "---" : String.format(Locale.US, "%.1f", fieldIn));
        mirroredLines.add("fieldDistanceIn: " + ((fieldIn == null) ? "---" : String.format(Locale.US, "%.1f", fieldIn)));

        String obeliskLine = ObeliskSignal.getDisplay();
        telemetry.addData("Obelisk", obeliskLine);
        mirroredLines.add("Obelisk: " + obeliskLine.replace("Obelisk: ", ""));

        String lockLine = tagLocked ? "LOCKED" : "SEARCHING";
        telemetry.addData("AprilTag Lock", lockLine);
        mirroredLines.add("AprilTag Lock: " + lockLine);
        if (feed != null) {
            if (feed.wasWindowLimitReached()) {
                String line = "FeedStop: scale window hit bounds – angles trimmed.";
                telemetry.addLine(line);
                mirroredLines.add(line);
            } else if (feed.wasAngleClamped()) {
                String line = "FeedStop: angles trimmed to fit available span.";
                telemetry.addLine(line);
                mirroredLines.add(line);
            }
            if (feed.wasSoftLimitClamped() && feed.getSoftLimitMessage() != null) {
                String line = feed.getSoftLimitMessage();
                telemetry.addLine(line);
                mirroredLines.add(line);
            }
            if (feed.wasHomeAborted() && feed.getHomeAbortMessage() != null) {
                String line = "FeedStop: " + feed.getHomeAbortMessage();
                telemetry.addLine(line);
                mirroredLines.add(line);
            }
            String summary = feed.getFeedStopSummaryLine();
            telemetry.addLine(summary);
            mirroredLines.add(summary);
        }
        if (DebugTelemetryConfig.TELEOP_TELEMETRY_DEBUG_ENABLED && firingController != null) {
            String feedStopState = (feed != null && feed.getFeedStopState() != null)
                    ? feed.getFeedStopState().name()
                    : "N/A";
            long msSinceRequest = firingController.getMsSinceLastRequest();
            String sinceRequestStr = (msSinceRequest >= 0L) ? String.valueOf(msSinceRequest) : "N/A";
            String profile = firingController.getFiringProfile();
            String burstLike = firingController.isBurstLike() ? "YES" : "NO";
            String leadMs = String.valueOf(firingController.getLeadMsApplied());
            String recoveryBand = String.format(Locale.US, "%.0f", firingController.getRecoveryBandUsed());
            String recoveryMaxMs = String.valueOf(firingController.getRecoveryMaxMsUsed());

            telemetry.addData("FeedStop State", feedStopState);
            telemetry.addData("Firing Profile", profile);
            telemetry.addData("Burst Like", String.format(Locale.US, "%s (%sms)", burstLike, sinceRequestStr));
            telemetry.addData("Lead Ms", leadMs);
            telemetry.addData("Recovery Band", recoveryBand);
            telemetry.addData("Recovery Max (ms)", recoveryMaxMs);
            mirroredLines.add("FeedStop State: " + feedStopState);
            mirroredLines.add("Firing Profile: " + profile);
            mirroredLines.add("Burst Like: " + burstLike + " (" + sinceRequestStr + "ms)");
            mirroredLines.add("Lead Ms: " + leadMs);
            mirroredLines.add("Recovery Band: " + recoveryBand);
            mirroredLines.add("Recovery Max (ms): " + recoveryMaxMs);
        }
        if (limelightAutoSelector != null) {
            String profileLine = limelightAutoSelector.getProfileLine();
            telemetry.addLine(profileLine);
            mirroredLines.add(profileLine);
            String memoryLine = limelightAutoSelector.getMemoryFallbackLine();
            if (memoryLine != null) {
                telemetry.addLine(memoryLine);
                mirroredLines.add(memoryLine);
            }
        }
        if (limelightAutoSelector != null && isStarted()) {
            String runningLine = limelightAutoSelector.getRunningStatusLine();
            if (runningLine != null) {
                telemetry.addLine(runningLine);
                mirroredLines.add(runningLine);
            }
        }
        sendDashboard(lastDashboardPose, statusPhase, mirroredLines);
    }

    private void seedOdometryFromPose(FieldPose pose, String reason, boolean fromVision) {
        if (odometry == null || pose == null) return;
        odometry.setPoseWithImuAlignment(pose.x, pose.y, pose.headingDeg);
        odometrySeedPose = odometry.getPose();
        seedImuYawDeg = odometry.getImuYawAtSeedDeg();
        seedHeadingOffsetDeg = odometry.getHeadingOffsetDeg();
        seedHeadingDeg = pose.headingDeg;
        visionSeededStart = fromVision;
        visionSeedReason = (reason == null || reason.isEmpty()) ? "startPose" : reason;
    }

    private void sendDashboard(FieldPose pose, String statusLabel, List<String> mirroredLines) {
        if (dashboard == null || pose == null) return;
        TelemetryPacket packet = new TelemetryPacket();
        if (mirroredLines != null) {
            for (String line : mirroredLines) {
                packet.addLine(line);
            }
        }
        DecodeFieldDrawing.drawField(packet, pose, alliance(), ObeliskSignal.get());
        dashboard.sendTelemetryPacket(packet);
    }

    private FieldPose saveFinalPoseForTeleOp() {
        return saveFinalPoseForTeleOp(null);
    }

    private FieldPose saveFinalPoseForTeleOp(FieldPose finalPose) {
        FieldPose resolved = (finalPose != null)
                ? finalPose
                : (odometry != null ? odometry.update() : startPose);
        try { PoseStore.setLastKnownPose(resolved); } catch (Throwable ignored) {}
        lastDashboardPose = resolved;
        return resolved;
    }

    private int allianceGoalTagId() {
        return (alliance() == Alliance.BLUE) ? VisionAprilTag.TAG_BLUE_GOAL : VisionAprilTag.TAG_RED_GOAL;
    }

    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
    private static double shortestDiff(double target, double current) {
        double d = normDeg(target - current); if (d > 180) d -= 360; if (d < -180) d += 360; return d;
    }
    private static double normDeg(double a) { double r = a % 360; if (r < 0) r += 360; return r; }

    private static final class LockWindow {
        final double minDeg;
        final double maxDeg;

        LockWindow(double minDeg, double maxDeg) {
            this.minDeg = minDeg;
            this.maxDeg = maxDeg;
        }

        boolean contains(double bearingDeg) {
            return bearingDeg >= minDeg && bearingDeg <= maxDeg;
        }
    }

    private LockWindow computeLockWindow(boolean longShotMode, double toleranceDeg) {
        double tol = Math.abs(toleranceDeg);
        if (!longShotMode) {
            return new LockWindow(-tol, tol);
        }
        return (alliance() == Alliance.RED)
                ? new LockWindow(-tol, 0.0)
                : new LockWindow(0.0, tol);
    }

    private boolean isLongShot(Double distanceIn) {
        if (!AutoAimTuning.LONG_SHOT_ENABLED) {
            return false;
        }
        return distanceIn != null && distanceIn >= AutoAimTuning.LONG_SHOT_DISTANCE_IN;
    }

    private Double distanceInchesFromProvider(VisionTargetProvider provider) {
        if (provider == null || !provider.isGoalVisibleSmoothed()) {
            return null;
        }
        double rangeM = provider.getDistanceMeters();
        if (Double.isNaN(rangeM) || !Double.isFinite(rangeM)) {
            return null;
        }
        return rangeM * M_TO_IN;
    }

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

    protected final void spinLauncherToAutoRpmDefault(String phase) {
        String label = (phase == null || phase.isEmpty()) ? "Spin to auto RPM" : phase;
        autoCtrl.setAutoEnabled(true);
        double seed = autoSeedRpm();
        try { autoCtrl.setDefaultRpm(seed); } catch (Throwable ignored) {}
        double target = autoCtrl.hold();
        if (target <= 0) { target = seed; }
        launcher.setTargetRpm(target);
        updateStatusWithPose(label, false, updateOdometryPose());
        telemetry.addData("Target RPM", target);
        telemetry.addData("Auto default RPM", seed);
        telemetry.update();
    }

    protected final AutoSequence sequence() {
        return new AutoSequence();
    }

    protected final class AutoSequence {
        private final List<AutoStep> steps = new ArrayList<>();
        private final List<AutoStep> endgameSteps = new ArrayList<>();
        private double storedHeading = Double.NaN;
        private boolean lastLock = false;
        private boolean lastAimReady = false;

        private AutoSequence addStep(AutoStep step) {
            steps.add(step);
            return this;
        }

        private AutoSequence addEndgameStepInternal(AutoStep step) {
            endgameSteps.add(step);
            return this;
        }

        private String resolveLabel(String provided, String fallback) {
            return (provided == null || provided.isEmpty()) ? fallback : provided;
        }

        private AutoStep buildMoveStep(String phase, double distanceInches, double headingDeg, double twistDeg, double speedCap) {
            return () -> {
                String label = resolveLabel(phase, "Move");
                lastLock = false;
                lastAimReady = false;
                double speed = clampTranslationSpeed(speedCap);
                double startHeading = drive.heading();
                double targetHeading = normDeg(startHeading + twistDeg);
                double absoluteHeading = normDeg(startHeading + headingDeg);
                double turnSpeed = clampTurnSpeed(speedCap);
                updateStatusWithPose(label, false, updateOdometryPose());
                telemetry.addData("Distance (in)", distanceInches);
                telemetry.addData("Heading offset (deg)", headingDeg);
                telemetry.addData("Resolved heading (deg)", absoluteHeading);
                telemetry.addData("Twist target (deg)", targetHeading);
                telemetry.addData("Start heading (deg)", startHeading);
                telemetry.addData("Speed cap", speed);
                telemetry.addData("Twist speed cap", turnSpeed);
                telemetry.update();
                drive.moveWithTwist(distanceInches, absoluteHeading, targetHeading, speed, turnSpeed);
                drive.stopAll();
                updateStatusWithPose(label + " complete", false, updateOdometryPose());
                telemetry.update();
            };
        }

        public AutoSequence rememberHeading(String phase) {
            return addStep(() -> {
                storedHeading = drive.heading();
                String label = resolveLabel(phase, "Record heading");
                updateStatusWithPose(label, lastLock, updateOdometryPose());
                telemetry.addData("Stored heading (deg)", storedHeading);
                telemetry.update();
            });
        }

        public AutoSequence move(String phase, double distanceInches, double headingDeg, double twistDeg, double speedCap) {
            return addStep(buildMoveStep(phase, distanceInches, headingDeg, twistDeg, speedCap));
        }

        public AutoSequence endgameMove(String phase, double distanceInches, double headingDeg, double twistDeg, double speedCap) {
            return addEndgameStepInternal(buildMoveStep(phase, distanceInches, headingDeg, twistDeg, speedCap));
        }

        public AutoSequence addEndgameStep(AutoStep step) {
            return addEndgameStepInternal(step);
        }

        public AutoSequence rotate(String phase, double degrees, double speedCap) {
            return addStep(() -> {
                String label = resolveLabel(phase, "Rotate");
                lastLock = false;
                lastAimReady = false;
                double speed = clampTurnSpeed(speedCap);
                updateStatusWithPose(label, false, updateOdometryPose());
                telemetry.addData("Delta (deg)", degrees);
                telemetry.addData("Speed cap", speed);
                telemetry.update();
                drive.turn(degrees, speed);
                updateStatusWithPose(label + " complete", false, updateOdometryPose());
                telemetry.update();
            });
        }

        public AutoSequence rotateToHeading(String phase, double headingDeg, double speedCap) {
            return addStep(() -> {
                String label = resolveLabel(phase, "Rotate to heading");
                lastLock = false;
                lastAimReady = false;
                double speed = clampTurnSpeed(speedCap);
                updateStatusWithPose(label, false, updateOdometryPose());
                telemetry.addData("Target heading (deg)", headingDeg);
                telemetry.addData("Speed cap", speed);
                telemetry.update();
                double delta = shortestDiff(headingDeg, drive.heading());
                drive.turn(delta, speed);
                updateStatusWithPose(label + " complete", false, updateOdometryPose());
                telemetry.update();
            });
        }

        public AutoSequence spinToAutoRpmDefault(String phase) {
            return addStep(() -> {
                lastLock = false;
                lastAimReady = false;
                spinLauncherToAutoRpmDefault(phase);
            });
        }

        @Deprecated
        public AutoSequence spinToAutoRpm(String phase) {
            telemetry.log().add("DEPRECATED AutoSequence.spinToAutoRpm(...) – use spinToAutoRpmDefault(...)");
            return spinToAutoRpmDefault(phase);
        }

        public AutoSequence rotateToTarget(String phase,
                                           ScanDirection direction,
                                           double turnSpeedFraction,
                                           double primarySweepDeg,
                                           double oppositeSweepDeg,
                                           long timeoutMs) {
            return addStep(() -> {
                lastAimReady = false;
                lastLock = turnToGoalTag(phase, direction, turnSpeedFraction, primarySweepDeg, oppositeSweepDeg, timeoutMs);
                if (!lastLock) {
                    telemetry.addLine("⚠️ No tag lock – continuing sequence");
                    telemetry.update();
                }
            });
        }

        public AutoSequence rotateToTarget(String phase,
                                           ScanDirection direction,
                                           double turnSpeedFraction,
                                           double primarySweepDeg,
                                           long timeoutMs) {
            return addStep(() -> {
                lastAimReady = false;
                lastLock = turnToGoalTag(phase, direction, turnSpeedFraction, primarySweepDeg, null, timeoutMs);
                if (!lastLock) {
                    telemetry.addLine("⚠️ No tag lock – continuing sequence");
                    telemetry.update();
                }
            });
        }

        public AutoSequence rotateToTarget(String phase,
                                           double turnSpeedFraction,
                                           double primarySweepDeg,
                                           double oppositeSweepDeg,
                                           long timeoutMs) {
            return rotateToTarget(phase, null, turnSpeedFraction, primarySweepDeg, oppositeSweepDeg, timeoutMs);
        }

        public AutoSequence rotateToTarget(String phase,
                                           double turnSpeedFraction,
                                           double primarySweepDeg,
                                           long timeoutMs) {
            return rotateToTarget(phase, null, turnSpeedFraction, primarySweepDeg, timeoutMs);
        }

        public AutoSequence visionMode(String phase, VisionTuning.Mode mode) {
            return addStep(() -> {
                String label = resolveLabel(phase, "Select vision profile");
                lastLock = false;
                lastAimReady = false;
                updateStatusWithPose(label, false, updateOdometryPose());

                VisionConfig.VisionSource source;
                try { source = VisionConfig.VISION_SOURCE; }
                catch (Throwable ignored) { source = VisionConfig.VisionSource.WEBCAM_LEGACY; }
                if (source == VisionConfig.VisionSource.LIMELIGHT) {
                    telemetry.addLine("Limelight vision active – skipping profile change");
                    telemetry.update();
                    return;
                }

                VisionTuning.Mode requested = (mode != null) ? mode : VisionTuning.DEFAULT_MODE;
                telemetry.addData("Requested mode", requested);

                if (vision == null) {
                    telemetry.addLine("⚠️ Vision unavailable – skipping profile change");
                } else {
                    try {
                        vision.applyProfile(requested);
                        try { vision.setRangeScale(VisionTuning.RANGE_SCALE); } catch (Throwable ignored) {}

                        VisionTuning.Mode activeMode = vision.getActiveMode();
                        VisionTuning.Profile profile = vision.getActiveProfile();
                        telemetry.addData("Active mode", activeMode);
                        if (profile != null) {
                            telemetry.addData("Resolution", String.format(Locale.US, "%dx%d@%dfps", profile.width, profile.height, profile.fps));
                            telemetry.addData("Decimation", profile.decimation);
                            telemetry.addData("Process every N", profile.processEveryN);
                        }
                    } catch (IllegalStateException ise) {
                        telemetry.addLine("⚠️ Vision profile error: " + ise.getMessage());
                    }
                }

                telemetry.update();
            });
        }

        public AutoSequence intake(String phase, boolean enabled) {
            return addStep(() -> {
                String label = resolveLabel(phase, enabled ? "Enable intake" : "Disable intake");
                lastLock = false;
                lastAimReady = false;
                updateStatusWithPose(label, false, updateOdometryPose());
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
                updateStatusWithPose(label, false, updateOdometryPose());
                telemetry.update();
                stopAll();
                updateStatusWithPose(label + " complete", false, updateOdometryPose());
                telemetry.update();
            });
        }

        public AutoSequence readyToLaunch(String phase, long timeoutMs) {
            return addStep(() -> {
                lastAimReady = readyLauncherUntilReady(timeoutMs, phase);
                if (lastReadyHadLock) {
                    lastLock = true;
                }
                if (!lastAimReady) {
                    telemetry.addLine("⚠️ Launcher not at speed before timeout");
                    telemetry.update();
                }
            });
        }

        @Deprecated
        public AutoSequence aim(String phase, long timeoutMs) {
            telemetry.log().add("DEPRECATED AutoSequence.aim(...) – use readyToLaunch(...)");
            return readyToLaunch(phase, timeoutMs);
        }

        public AutoSequence waitFor(String phase, long milliseconds) {
            return addStep(() -> {
                String label = resolveLabel(phase, "Wait");
                updateStatusWithPose(label, lastLock, updateOdometryPose());
                telemetry.addData("Duration (ms)", milliseconds);
                telemetry.update();
                sleep(milliseconds);
            });
        }

        public AutoSequence fire(String phase, int shots, boolean requireLock, boolean requireLauncherAtSpeed, long betweenShotsMs) {
            return addStep(() -> {
                String label = resolveLabel(phase, "Fire");
                if (requireLock && !lastLock) {
                    updateStatusWithPose(label + " – skipped (no lock)", false, updateOdometryPose());
                    telemetry.addLine("⚠️ Fire skipped because tag lock was not achieved");
                    telemetry.update();
                    return;
                }
                updateStatusWithPose(label, !requireLock || lastLock, updateOdometryPose());
                telemetry.addData("Shots", shots);
                telemetry.addData("Lock required", requireLock);
                telemetry.addData("RPM required", requireLauncherAtSpeed);
                telemetry.addData("Between shots (ms)", betweenShotsMs);
                telemetry.update();
                fireN(shots, requireLock, requireLauncherAtSpeed, betweenShotsMs);
                lastAimReady = false;
                lastLock = requireLock && lastLock;
            });
        }

        public AutoSequence fireContinuous(String phase, long durationMs, boolean requireLock, boolean requireLauncherAtSpeed) {
            return addStep(() -> {
                String label = resolveLabel(phase, "Continuous fire");
                boolean lockSatisfied = lastLock || lastReadyHadLock;
                if (requireLock && !lockSatisfied) {
                    updateStatusWithPose(label + " – skipped (no lock)", false, updateOdometryPose());
                    telemetry.addLine("⚠️ Continuous fire skipped because tag lock was not achieved");
                    telemetry.update();
                    return;
                }
                updateStatusWithPose(label, !requireLock || lockSatisfied, updateOdometryPose());
                telemetry.addData("Duration (ms)", durationMs);
                telemetry.addData("Lock required", requireLock);
                telemetry.addData("RPM required", requireLauncherAtSpeed);
                telemetry.update();
                BaseAuto.this.fireContinuous(durationMs, requireLock, requireLauncherAtSpeed, label);
                lastAimReady = false;
                lastLock = requireLock && lockSatisfied;
            });
        }

        public AutoSequence eject(String phase) {
            return addStep(() -> {
                String label = resolveLabel(phase, "Eject artifact");
                lastLock = false;
                lastAimReady = false;

                updateStatusWithPose(label, false, updateOdometryPose());
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
                feed.update();
                sleep(ejectDuration);
                feed.update();
                if (!intakeWasOn) {
                    sleep(FeedTuning.INTAKE_ASSIST_MS);
                    intake.set(false);
                    feed.update();
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

                updateStatusWithPose(label + " complete", false, updateOdometryPose());
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
                updateStatusWithPose(label, false, updateOdometryPose());
                telemetry.addData("Target heading (deg)", target);
                telemetry.addData("Speed cap", speed);
                telemetry.update();
                double delta = shortestDiff(target, drive.heading());
                drive.turn(delta, speed);
                updateStatusWithPose(label + " complete", false, updateOdometryPose());
                telemetry.update();
            });
        }

        public AutoSequence custom(AutoStep step) {
            return addStep(step);
        }

        public void run() throws InterruptedException {
            for (AutoStep step : steps) {
                if (!opModeIsActive()) {
                    return;
                }
                if (mainPhaseOver()) {
                    break;
                }
                step.run();
                if (mainPhaseOver()) {
                    break;
                }
            }
            if (!opModeIsActive()) {
                return;
            }
            for (AutoStep step : endgameSteps) {
                if (!opModeIsActive()) {
                    break;
                }
                step.run();
            }
        }
    }

    protected void updateIntakeFlowForAuto() {
        if (intake == null) {
            return;
        }
        boolean feedActive = (firingController != null && firingController.isFeedActive())
                || ((feed != null) && feed.isFeedCycleActive());
        intake.update(feedActive);
    }

    @FunctionalInterface
    protected interface AutoStep {
        void run() throws InterruptedException;
    }
}
