/*
 * FILE: TeleOpAllianceBase.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleop/
 *
 * PURPOSE
 *   - Serve as the shared TeleOp core for both alliances—handling driver input,
 *     drivetrain control, launcher automation, AprilTag aim/auto-speed assists,
 *     rumble feedback, and endgame safety.
 *   - Provide one place for students to update driver workflow so TeleOp_Blue
 *     and TeleOp_Red stay aligned.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md tables for ranges + overrides)
 *   - DEFAULT_AUTOSPEED_ENABLED / DEFAULT_AUTOAIM_ENABLED / DEFAULT_INTAKE_ENABLED
 *       • Driver defaults on init. TeleOp-only; Autonomous ignores these.
 *   - slowestSpeed
 *       • Minimum drive power when the brake trigger is held (Intake power &
 *         driver defaults table).
 *   - rpmBottom / rpmTop
 *       • Manual RPM slider bounds when AutoSpeed is off. Ensure rpmTop ≤
 *         Launcher.RPM_MAX.
 *   - autoAimLossGraceMs
 *       • Grace period before AutoAim disengages when tags drop.
 *   - SMOOTH_A
 *       • Telemetry smoothing constant for range/heading displays.
 *   - aimRumble* + togglePulse*
 *       • Haptic envelopes for aim window + toggle feedback (Driver feedback table).
 *   - ejectRpm / ejectTimeMs
 *       • TeleOp-only eject routine behavior.
 *   - InitialAutoDefaultSpeed
 *       • Local override of SharedRobotTuning.INITIAL_AUTO_DEFAULT_SPEED. Update
 *         assist/AutoAimSpeed.initialAutoDefaultSpeed when diverging.
 *   - intakeAssistMs
 *       • TeleOp-specific copy of SharedRobotTuning.INTAKE_ASSIST_MS for post-shot
 *         intake run.
 *   - autoStopTimerEnabled / autoStopTimerTimeSec
 *       • Optional endgame safety timer configuration.
 *
 * METHODS
 *   - init()
 *       • Initialize subsystems, vision, rumble profiles, controller bindings,
 *         and copies of shared tuning constants.
 *   - loop()
 *       • Run driver control logic each cycle—AutoAim/AutoSpeed, safety gating,
 *         telemetry updates, and rumble feedback.
 *   - stop()
 *       • Ensure all subsystems and vision resources shut down cleanly.
 *   - Helper sections (feedOnceWithIntakeAssist, handleRumble, applyDrive, etc.)
 *       • Group related logic for student readability.
 *
 * NOTES
 *   - TeleOp_Blue / TeleOp_Red only supply alliance(); any behavioral change
 *     should live here so both inherit it.
 *   - SharedRobotTuning and AutoRpmConfig remain the authoritative sources for
 *     shared tunables—update those before tweaking the local copies below.
 *
 * CHANGES (2025-12-20): Split goal detection telemetry from aim validity for
 *                       Limelight auto-aim, adding raw goal tx validity and
 *                       observed ID reporting so drivers can see when the goal
 *                       tag is detected even if heading is unusable.
 * CHANGES (2025-12-17): Surfaced per-fiducial Limelight aim lock telemetry
 *                       (locked tx vs. global tx, lock freshness, smoothed
 *                       visibility) so drivers can confirm goal-only aiming
 *                       no longer jumps to obelisk detections.
 * CHANGES (2025-11-29): Surfaced AutoRPM tweak telemetry (D-pad left/right while
 *                       AutoSpeed is active) with percentage and RPM deltas so
 *                       drivers can see the live nudge under the RPM target
 *                       line.
 * CHANGES (2025-11-27): Repaired FTC Dashboard vision profile swap cleanup
 *                       braces so telemetry packets compile and send
 *                       correctly during TeleOp loops.
 * CHANGES (2025-11-26): Seed odometry from the last Auto pose when available
 *                       via the pose store, emit telemetry confirming the
 *                       handoff, and leave the stored pose untouched when
 *                       absent so TeleOp can still rely on AprilTag seeding
 *                       during INIT.
 * CHANGES (2025-11-26): Added FTC Dashboard telemetry + field overlays each loop
 *                       using DecodeFieldDrawing with the corrected +X/+Y frame,
 *                       artifact rows, and launch triangles so drivers can see
 *                       live odometry motion on the Dashboard alongside phone
 *                       telemetry.
 * CHANGES (2025-11-25): Added odometry carryover via PoseStore plus AprilTag
 *                       re-localization during INIT so TeleOp starts with a
 *                       continuous fused pose for dashboard overlays.
 * CHANGES (2025-12-03): Surfaced vision lighting telemetry (mean/alpha/beta +
 *                       adaptive state) from the new normalization layer so
 *                       drivers can tune brightness on dark/bright fields.
 * CHANGES (2025-12-03): Added goal-tag visibility tiers (raw/aim/smoothed),
 *                       vision health telemetry, and normalized-preview
 *                       plumbing for diagnostics while keeping AutoAim gating
 *                       aligned with smoothed visibility.
 * CHANGES (2025-12-03): Mirrored driver-station telemetry onto FTC Dashboard
 *                       with graphable launcher RPM channels and made long-shot
 *                       mode sticky until a new distance reading arrives so
 *                       brief tag dropouts no longer flip the shot window.
 * CHANGES (2025-12-16): Reversed long-shot lock bias so RED favors negative
 *                       bearings and BLUE favors positive bearings when the
 *                       range cutover engages in TeleOp aim windows.
 * CHANGES (2025-12-09): Dashboard packets now mirror only the driver-station
 *                       telemetry lines (no dashboard-only metrics) while
 *                       keeping field overlays; Obelisk scanning now falls back
 *                       to raw detections so motifs latch even when filtered
 *                       frames drop below the margin gate.
 * CHANGES (2025-12-03): AutoAim + AutoSpeed now rely solely on the
 *                       alliance-correct goal tag; non-alliance tags remain
 *                       limited to odometry blending so shooter targets stay
 *                       aligned to field scoring intent.
 *
 * CHANGES (2025-11-22): Added a tunable master switch for long-shot lock biasing
 *                       so crews can revert to symmetric windows without code
 *                       changes.
 * CHANGES (2025-11-20): Require a live AprilTag sighting to enter long-shot
 *                       lock biasing so asymmetric tolerances only engage when
 *                       distance comes from the current detection.
 * CHANGES (2025-11-19): Reordered top-line telemetry and split left/right RPM
 *                       readings so drivers can monitor each flywheel.
 * CHANGES (2025-11-18): Bias AutoAim deadband toward the alliance-correct side
 *                       when shooting from long range using the new long-shot
 *                       distance cutover.
 * CHANGES (2025-11-15): AutoRPM telemetry now summarizes the calibration table
 *                       (point count + endpoint pairs) so drivers see the
 *                       config-driven curve without plugging into code.
 * CHANGES (2025-11-25): Added a triple-tap RB gesture that runs an intake reverse
 *                       pulse without disrupting the existing intake toggle.
 * CHANGES (2025-11-25): Latched the triple-tap RB gesture into a continuous reverse
 *                       run that ends on the next tap, restoring the saved intake
 *                       toggle state when released and aligned changelog dates
 *                       with the 2025-11-25 release.
 * CHANGES (2025-11-14): Intake assist restore now re-applies the driver's
 *                       pre-shot state after the timer instead of latching the
 *                       intake ON when it was manually disabled before the
 *                       feed.
 * CHANGES (2025-11-12): StopAll now caches the live intake state so resuming
 *                       with Start restores the previous ON/OFF setting
 *                       instead of forcing the intake off until retoggled.
 * CHANGES (2025-11-10): Added a Reverse Drive toggle on Gamepad 1 left stick
 *                       click that inverts forward/strafe commands and emits
 *                       double/single rumble on enable/disable for driver
 *                       confirmation.
 * CHANGES (2025-10-31): Added runtime vision profile + live-view switching on
 *                       Gamepad 2 D-pad, refreshed telemetry with concise
 *                       profile/perf lines, and kept camera control warnings
 *                       throttled for driver readability.
 * CHANGES (2025-10-31): Disabled feed idle hold while StopAll is latched so the
 *                       motor stays at 0 power, restoring the hold when Start
 *                       resumes TeleOp control.
 * CHANGES (2025-12-11): Wired TeleOp to select a VisionTargetProvider (Limelight
 *                       default, webcam legacy fallback) via VisionConfig and
 *                       feed it into aim/auto-speed helpers.
 * CHANGES (2025-12-11): Recentered default odometry pose on the field-center
 *                       frame (human wall = −72" Y) and moved odometry fusion
 *                       to Limelight-only XY blending with bounded corrections
 *                       (IMU-only heading, no webcam pose fusion).
 * CHANGES (2025-12-17): Restored the driver’s AutoAim toggle after releasing a
 *                       continuous-feed hold so the temporary shot assist no
 *                       longer latches AutoAim on once streaming stops.
*/
package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.drive.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Feed;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.odometry.DecodeFieldDrawing;
import org.firstinspires.ftc.teamcode.odometry.FieldPose;
import org.firstinspires.ftc.teamcode.odometry.Odometry;
import org.firstinspires.ftc.teamcode.odometry.PoseStore;

// === VISION IMPORTS ===
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.config.VisionConfig;
import org.firstinspires.ftc.teamcode.config.VisionConfig.VisionSource;
import org.firstinspires.ftc.teamcode.vision.LimelightTargetProvider;
import org.firstinspires.ftc.teamcode.vision.TagAimController;
import org.firstinspires.ftc.teamcode.vision.VisionAprilTag;
import org.firstinspires.ftc.teamcode.vision.VisionTargetProvider;
import org.firstinspires.ftc.teamcode.vision.WebcamLegacyTargetProvider;
import org.firstinspires.ftc.teamcode.utils.ObeliskSignal;

import static java.lang.Math.*;

// === CONTROLLER BINDINGS ===
import org.firstinspires.ftc.teamcode.input.ControllerBindings;
import static org.firstinspires.ftc.teamcode.input.ControllerBindings.Pad;
import static org.firstinspires.ftc.teamcode.input.ControllerBindings.Btn;
import static org.firstinspires.ftc.teamcode.input.ControllerBindings.Trigger;

// === HAPTICS (RUMBLE) ===
import org.firstinspires.ftc.teamcode.util.RumbleNotifier;

// === AUTO LAUNCHER SPEED (RPM) ===
import org.firstinspires.ftc.teamcode.control.LauncherAutoSpeedController;

// === CENTRALIZED TUNING (NEW) ===
// Where these used to live:
//  - AutoRPM curve in this file: autoNearDistIn/autoNearRpm/autoFarDistIn/autoFarRpm/autoSmoothingAlpha
//  - Intake Assist & InitialAutoDefaultSpeed in this file
// Now update them in these configs instead:
import org.firstinspires.ftc.teamcode.config.AutoAimTuning;
import org.firstinspires.ftc.teamcode.config.AutoRpmConfig;      // distance→RPM curve + smoothing
import org.firstinspires.ftc.teamcode.config.LauncherTuning;
import org.firstinspires.ftc.teamcode.config.OdometryConfig;
import org.firstinspires.ftc.teamcode.config.TeleOpDriverDefaults; // TeleOp-only workflow + manual ranges
import org.firstinspires.ftc.teamcode.config.TeleOpEjectTuning;    // TeleOp-only eject routine
import org.firstinspires.ftc.teamcode.config.TeleOpRumbleTuning;   // Driver rumble envelopes
import org.firstinspires.ftc.teamcode.config.TagAimTuning;
import org.firstinspires.ftc.teamcode.config.VisionTuning;         // AprilTag range calibration

import java.util.Locale;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.RejectedExecutionException;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public abstract class TeleOpAllianceBase extends OpMode {
    // CHANGES (2025-10-30): Added AutoAim drive speed scaling, manual RPM D-pad nudges (AutoSpeed off & lock engaged), and telemetry updates.
    // CHANGES (2025-10-31): Added safeInit gating and defaulted AutoSpeed + intake to ON after START.
    // CHANGES (2025-11-04): StopAll now enforces BRAKE zero-power behavior across drive, launcher, feed, and intake.
    // CHANGES (2025-11-07): Made feed/eject routines asynchronous with intake-assist timers so driver
    //                       controls remain responsive during shots, and reworked toggle rumble pulses
    //                       to queue the second blip without sleeping the TeleOp thread.
    // CHANGES (2025-11-07): Queue vision profile swaps on a background executor so TeleOp drive
    //                       updates continue while the VisionPortal rebuilds when drivers change modes.
    // CHANGES (2025-11-07): Queue AutoSpeed enable/disable requests so seeding RPM + rumble feedback
    //                       execute after the control scan without stalling drive inputs.
    // CHANGES (2025-11-07): Surface FeedStop homing status/telemetry and run init_loop updates so zero
    //                       is established before START without blocking the thread.
    // CHANGES (2025-11-09): Condensed FeedStop telemetry to a single summary line plus warnings only
    //                       when clamping or homing guards trigger.
    // CHANGES (2025-11-16): Integrated the encoder-aware intake flow classifier so TeleOp updates it
    //                       every loop and surfaces the phase in telemetry.
    // CHANGES (2025-11-23): Added AutoRPM tweak scaling (D-pad left/right while AutoSpeed enabled) and
    //                       continuous-feed hold behavior on the fire button.
    // CHANGES (2025-11-29): Restored single-shot tap behavior by delaying continuous-feed conversion
    //                       until after the FeedStop release window and added a temporary auto-aim
    //                       nudge before firing whenever a goal tag is visible, restoring the prior
    //                       AutoAim state afterward.
    // CHANGES (2025-12-02): Allow tap-to-fire to work even when the FeedStop release hold window is
    //                       configured to 0 ms by gating the single-shot block on a nonzero release
    //                       window while keeping continuous holds immediate when enabled.
    protected abstract Alliance alliance();

    // ---------------- Startup Defaults (edit here) ----------------
    private static final boolean DEFAULT_AUTOSPEED_ENABLED = TeleOpDriverDefaults.AUTO_SPEED_ENABLED; // TeleOp default; Auto ignores this flag
    private static final boolean DEFAULT_AUTOAIM_ENABLED   = TeleOpDriverDefaults.AUTO_AIM_ENABLED;   // TeleOp default for aim assist
    private static final boolean DEFAULT_INTAKE_ENABLED    = TeleOpDriverDefaults.INTAKE_ENABLED;     // Whether TeleOp begins with intake running

    // ---------------- Subsystems ----------------
    protected Drivebase drive;
    protected Launcher launcher;
    protected Feed feed;
    protected Intake intake;

    // ---------------- Drivetrain Tunables ----------------
    private double slowestSpeed = TeleOpDriverDefaults.SLOWEST_SPEED; // Min drive power while brake trigger held

    // ---------------- Launcher Manual Range (used when AutoSpeed == false) ----------------
    private double rpmBottom    = TeleOpDriverDefaults.RPM_BOTTOM; // Manual RPM lower bound when AutoSpeed is off
    private double rpmTop       = TeleOpDriverDefaults.RPM_TOP;    // Manual RPM upper bound; keep ≤ Launcher.RPM_MAX
    private double manualRpmStep = LauncherTuning.MANUAL_RPM_STEP; // Manual D-pad adjustment size when AutoSpeed is off & locked

    // ---------------- State ----------------
    private boolean autoSpeedEnabled = DEFAULT_AUTOSPEED_ENABLED; // Live state toggled by drivers
    private enum AutoSpeedRumble { NONE, SINGLE, DOUBLE }
    private boolean autoSpeedTogglePending = false;
    private boolean autoSpeedToggleTarget = false;
    private boolean autoSpeedToggleStopOnDisable = false;
    private Gamepad autoSpeedTogglePad = null;
    private AutoSpeedRumble autoSpeedToggleRumble = AutoSpeedRumble.NONE;
    private boolean autoAimEnabled   = DEFAULT_AUTOAIM_ENABLED;   // Live state for AprilTag aim assist
    private boolean longShotMode     = false;                     // Sticky long/normal shot window selection

    // AutoRPM tweak (per-press percentage scaling while AutoSpeed is enabled)
    private double autoRpmTweakScale  = TeleOpDriverDefaults.AUTORPM_TWEAK_SCALE;
    private double autoRpmTweakFactor = 1.0; // Multiplier applied to AutoSpeed outputs (x1.00 default)

    // Manual RPM Lock (Square/X) — only when AutoSpeed == false
    private boolean manualRpmLocked = false; // Manual RPM hold toggle (Square/X) when AutoSpeed disabled
    private double  manualLockedRpm = 0.0;   // Stored RPM when manualRpmLocked is true

    // ---------------- Odometry ----------------
    private Odometry odometry;
    private FieldPose fusedPose = new FieldPose(0.0, OdometryConfig.HUMAN_WALL_Y, 0.0);
    private boolean poseSeeded = false;
    private FtcDashboard dashboard;

    // ---------------- Vision + Aim ----------------
    private VisionTargetProvider visionTargetProvider; // Selected heading/range provider (Limelight default)
    private Limelight3A limelight;                    // Limelight device when selected as the provider source
    private VisionAprilTag vision;                // Legacy AprilTag pipeline (only when webcam provider is selected)
    private TagAimController aim = new TagAimController(); // PD twist helper; TeleOp clamps via SharedRobotTuning
    private double autoAimSpeedScale = AutoAimTuning.AUTO_AIM_SPEED_SCALE; // Translation multiplier while AutoAim is active
    private long lastVisionTelemetryMs = 0L;      // Throttle (~10 Hz) for vision status lines
    private String visionStatusLine = "Vision: Profile=-- LiveView=OFF Res=---@-- Decim=-.- ProcN=1 MinM=--";
    private String visionPerfLine = "Perf: FPS=--- LatMs=---";
    private String visionLightingLine = null;
    private String visionHealthLine = null;
    private String limelightStatusLine = null;
    private String limelightHealthLine = null;
    private boolean visionWarningShown = false;
    private ExecutorService visionTaskExecutor;
    private volatile boolean visionProfileSwapInProgress = false;
    private volatile VisionTuning.Mode visionProfileSwapMode = null;
    private volatile String visionProfileError = null;
    private long visionHealthWindowStartMs = 0L;
    private int visionHealthSamples = 0;
    private int visionHealthGoodSamples = 0;
    private double visionHealthMarginSum = 0.0;
    private int visionHealthMarginCount = 0;
    private double visionHealthBrightnessSum = 0.0;
    private int visionHealthBrightnessCount = 0;

    // ---------------- AutoAim Loss Grace (CONFIGURABLE) ----------------
    private int  autoAimLossGraceMs = TeleOpDriverDefaults.AUTO_AIM_LOSS_GRACE_MS; // Grace period to reacquire tag before disabling AutoAim
    private long aimLossStartMs = -1L;      // Negative when not currently timing a loss window

    // ---------------- Pose/Range Smoothing (SCALED meters) ----------------
    private Double smHeadingDeg = null;               // Telemetry-smoothed heading (deg)
    private Double smRangeMeters = null;              // Telemetry-smoothed range (m)
    private double smoothA = TeleOpDriverDefaults.TELEMETRY_SMOOTH_A; // Low-pass smoothing constant for telemetry displays
    private static final double M_TO_IN = 39.37007874015748; // Conversion factor (meters→inches)

    // ---------------- Controller Bindings ----------------
    private ControllerBindings controls;              // Centralized driver bindings helper

    // ---------------- Reverse Drive Mode ----------------
    private boolean reverseDriveMode = false;         // Treat rear as the front when true

    // ---------------- RPM Test Mode ----------------
    private boolean rpmTestEnabled = false; // Manual RPM sweep test (D-pad adjustments)
    private double  rpmTestTarget  = 0.0;   // Current manual test RPM when enabled

    // ---------------- Twist Sign Test ----------------
    private boolean twistSignTestMode = false;   // Forces a constant +rotation to confirm drivebase sign
    private boolean lastTwistSignTestButton = false;

    // ---------------- Intake Resume ----------------
    private boolean intakeResumeState = DEFAULT_INTAKE_ENABLED; // Stored intake state for StopAll resume

    // ---------------- Aim Rumble (Haptics) ----------------
    private RumbleNotifier aimRumbleDriver1;          // Shared notifier handling all rumble envelopes
    private boolean aimRumbleEnabled       = TeleOpRumbleTuning.AIM_RUMBLE_ENABLED; // Master enable/disable for aim rumble cues
    private double  aimRumbleDeg           = TeleOpRumbleTuning.AIM_THRESHOLD_DEG;   // Heading error (deg) that begins rumble
    private double  aimRumbleMinStrength   = TeleOpRumbleTuning.AIM_STRENGTH_MIN;    // Lower bound rumble strength while in window
    private double  aimRumbleMaxStrength   = TeleOpRumbleTuning.AIM_STRENGTH_MAX;    // Upper bound rumble strength while in window
    private int     aimRumbleMinPulseMs    = TeleOpRumbleTuning.AIM_PULSE_MIN_MS;    // Minimum rumble pulse length (ms)
    private int     aimRumbleMaxPulseMs    = TeleOpRumbleTuning.AIM_PULSE_MAX_MS;    // Maximum rumble pulse length (ms)
    private int     aimRumbleMinCooldownMs = TeleOpRumbleTuning.AIM_COOLDOWN_MIN_MS; // Min cooldown between pulses (ms)
    private int     aimRumbleMaxCooldownMs = TeleOpRumbleTuning.AIM_COOLDOWN_MAX_MS; // Max cooldown between pulses (ms)

    // ---------------- Toggle Pulse Settings ----------------
    private double togglePulseStrength     = TeleOpRumbleTuning.TOGGLE_STRENGTH; // Haptic strength for toggle confirmation pulses
    private int    togglePulseStepMs       = TeleOpRumbleTuning.TOGGLE_STEP_MS;  // Duration of each pulse step (ms)
    private int    togglePulseGapMs        = TeleOpRumbleTuning.TOGGLE_GAP_MS;   // Gap between pulse steps (ms)

    // Queued second-stage rumble pulses (per gamepad)
    private boolean doublePulseQueuedG1    = false;
    private long    doublePulseAtMsG1      = 0L;
    private boolean doublePulseQueuedG2    = false;
    private long    doublePulseAtMsG2      = 0L;

    // ---------------- Auto Launcher Speed (RPM) ----------------
    private LauncherAutoSpeedController autoCtrl;     // Shared AutoSpeed helper fed by AutoRpmConfig

    // ---------------- AutoSpeed seeding behavior ----------------
    // (This used to be local: InitialAutoDefaultSpeed; now centralized)
    private double InitialAutoDefaultSpeed = TeleOpDriverDefaults.INITIAL_AUTO_DEFAULT_SPEED; // Local override of seed RPM
    private boolean autoHadTagFix = false; // Tracks whether AutoSpeed has seen at least one tag this enable cycle

    // ---------------- Intake Assist + Eject ----------------
    // (intakeAssistMs used to be local; now driven by SharedRobotTuning)
    private int    intakeAssistMs = TeleOpDriverDefaults.INTAKE_ASSIST_MS; // TeleOp copy of shared intake assist duration
    private double ejectRpm       = TeleOpEjectTuning.RPM;   // Launcher RPM during eject routine (TeleOp only)
    private int    ejectTimeMs    = TeleOpEjectTuning.TIME_MS;   // Duration of eject routine (ms)

    private boolean intakeAssistRestorePending = false;
    private boolean intakeAssistWaitingForFeed = false;
    private boolean intakeAssistTargetState = false;
    private long intakeAssistResumeAtMs = 0L;
    private long intakeAssistExtraHoldMs = 0L;
    private boolean intakeAssistSawFeedActive = false;
    private int intakeReverseTapWindowMs = TeleOpDriverDefaults.INTAKE_REVERSE_TAP_WINDOW_MS;
    private int intakeReverseTapCount = 0;
    private long intakeReverseWindowStartMs = 0L;
    private boolean intakeReverseStartState = false;

    private boolean continuousFireHeld = false;   // True while LB is held for continuous feed
    private boolean continuousFireActive = false; // Latched once continuous feed has been enabled
    private long continuousFireHoldStartMs = 0L;  // Timestamp when the LB hold began for continuous feed
    private boolean pendingAutoAimNudge = false;  // Requests a temporary auto-aim before firing
    private boolean autoAimNudgeActive = false;   // AutoAim temporarily enabled for a shot
    private boolean autoAimNudgeRestoreState = false; // Previous AutoAim state to restore after the nudge

    private enum EjectPhase { IDLE, SPOOL, FEED, HOLD }
    private EjectPhase ejectPhase = EjectPhase.IDLE;
    private long ejectPhaseUntilMs = 0L;
    private double ejectPrevRpmCommand = 0.0;
    private boolean ejectFeedStarted = false;

    // ---------------- NEW: StopAll / Latch & Auto-Stop Timer ----------------
    /** When true, all outputs are forced to zero every loop until Start is pressed again. */
    private boolean stopLatched = false; // When true, StopAll has latched and outputs remain zeroed

    /** Auto-Stop timer master enable (defaults false). When true, shows top-line countdown and calls StopAll at 0. */
    protected boolean autoStopTimerEnabled = TeleOpDriverDefaults.AUTO_STOP_TIMER_ENABLED; // Optional endgame auto-stop timer flag

    /** Auto-Stop timer seconds from TeleOp INIT (defaults 119). */
    protected int autoStopTimerTimeSec = TeleOpDriverDefaults.AUTO_STOP_TIMER_TIME_SEC; // Default countdown seconds when timer enabled

    /** Timestamp captured when TeleOp STARTS; used as the timer start. */
    private long teleopInitMillis = 0L; // TeleOp start timestamp for timer calculations

    /** Ensures the timer only trips StopAll once at expiry. */
    private boolean autoStopTriggered = false; // Ensures we only stop once when timer expires

    /** Debounce for raw Start-button edge detection while STOPPED (works even when controls callbacks are bypassed). */
    private boolean lastStartG1 = false, lastStartG2 = false; // Debounce Start-button edges for stop latch release

    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }

    @Override
    public void init() {
        // ---- Subsystem Initialization ----
        drive    = new Drivebase(hardwareMap, telemetry);
        launcher = new Launcher(hardwareMap);
        feed     = new Feed(hardwareMap);
        intake   = new Intake(hardwareMap);
        drive.safeInit();
        launcher.safeInit();
        feed.safeInit();
        intake.safeInit();
        feed.initFeedStop(hardwareMap, telemetry);

        if (visionTaskExecutor != null) {
            visionTaskExecutor.shutdownNow();
        }
        visionTaskExecutor = Executors.newSingleThreadExecutor(r -> {
            Thread t = new Thread(r, "TeleOpVisionTasks");
            t.setDaemon(true);
            return t;
        });
        visionProfileSwapInProgress = false;
        visionProfileSwapMode = null;
        visionProfileError = null;

        // ---- Vision Initialization ----
        vision = null;
        limelight = null;
        visionTargetProvider = null;
        VisionSource source = VisionConfig.VISION_SOURCE;
        if (source == VisionSource.WEBCAM_LEGACY) {
            vision = new VisionAprilTag();
            vision.init(hardwareMap, "Webcam 1");
            vision.setGoalVisibilityTargetId(allianceGoalTagId());
            vision.setRangeScale(VisionTuning.RANGE_SCALE); // keep your calibration scale unless re-tuned
            visionTargetProvider = new WebcamLegacyTargetProvider(vision, this::alliance);
        } else {
            try {
                limelight = hardwareMap.get(Limelight3A.class, "limelight");
            } catch (Exception ex) {
                telemetry.addLine("Limelight not found; ensure EthernetDevice named 'limelight'");
            }

            if (limelight != null) {
                applyLimelightPipeline(limelight);
                applyLimelightPollRate(limelight);
                try { limelight.start(); } catch (Throwable ignored) {}
            }

            visionTargetProvider = new LimelightTargetProvider(limelight, this::alliance);
        }

        aim.setProvider(visionTargetProvider);
        lastVisionTelemetryMs = 0L;
        visionWarningShown = false;
        visionStatusLine = "Vision: Profile=-- LiveView=OFF Res=---@-- Decim=-.- ProcN=1 MinM=--";
        visionPerfLine = "Perf: FPS=--- LatMs=---";

        odometry = new Odometry(drive, limelight);
        FieldPose storedPose = PoseStore.consumeLastKnownPose();
        if (storedPose != null) {
            fusedPose = storedPose;
            odometry.setPose(storedPose.x, storedPose.y, storedPose.headingDeg);
            poseSeeded = true;
            telemetry.addLine(String.format("INIT pose from Auto: x=%.1f y=%.1f hdg=%.1f", storedPose.x, storedPose.y, storedPose.headingDeg));
        } else {
            odometry.setPose(fusedPose.x, fusedPose.y, fusedPose.headingDeg);
        }

        dashboard = FtcDashboard.getInstance();

        resetTogglePulseQueue();

        // ---- Controller Bindings Setup ----
        controls = new ControllerBindings();

        // =========================================================================
        // ========== CONTROLLER BINDINGS: ALL MAPPINGS CENTRALIZED (EDIT HERE) ====
        // =========================================================================

        // -------- Gamepad 1 (Driver) --------
        // Feed / Intake
        controls.bindPress(Pad.G1, Btn.LB, () -> feedOnceWithIntakeAssist());
        controls.bindHold(Pad.G1, Btn.LB, this::markContinuousFireHeld);
        controls.bindPress(Pad.G1, Btn.RB, this::handleIntakeButtonPress);

        // Reverse Drive toggle
        controls.bindPress(Pad.G1, Btn.L_STICK_BTN, this::toggleReverseDriveMode);

        // AutoAim toggle (gated by current tag visibility)
            controls.bindPress(Pad.G1, Btn.R_STICK_BTN, () -> {
                boolean hasGoal = visionTargetProvider != null && visionTargetProvider.hasGoalTarget();
                if (!autoAimEnabled) {
                    if (hasGoal) {
                        autoAimEnabled = true;
                    aimLossStartMs = -1;
                    pulseDouble(gamepad1);
                } else {
                    pulseSingle(gamepad1); // not available
                }
            } else {
                autoAimEnabled = false;
                aimLossStartMs = -1;
                pulseSingle(gamepad1);
            }
        });

        // AutoSpeed toggle (seed RPM if tag; else use InitialAutoDefaultSpeed)
        controls.bindPress(Pad.G1, Btn.Y, () -> {
            boolean enable = !autoSpeedEnabled;
            queueAutoSpeedEnablement(enable, /*stopOnDisable=*/false, gamepad1,
                    enable ? AutoSpeedRumble.DOUBLE : AutoSpeedRumble.SINGLE);
        });

        // Manual RPM LOCK (X/Square) — only when AutoSpeed == false & not in Test
        controls.bindPress(Pad.G1, Btn.X, () -> {
            if (!autoSpeedEnabled && !rpmTestEnabled) {
                if (!manualRpmLocked) {
                    manualRpmLocked = true;
                    manualLockedRpm = launcher.targetRpm;
                    pulseDouble(gamepad1);
                } else {
                    manualRpmLocked = false;
                    pulseSingle(gamepad1);
                }
            }
        });

        // EJECT (B/Circle)
        controls.bindPress(Pad.G1, Btn.B, () -> ejectOnce());

        // RPM TEST MODE (D-Pad)
        controls.bindPress(Pad.G1, Btn.DPAD_UP,    () -> { rpmTestEnabled = true;  launcher.setTargetRpm(rpmTestTarget); });
        controls.bindPress(Pad.G1, Btn.DPAD_LEFT,  () -> {
            if (rpmTestEnabled) {
                double lo = Math.max(rpmBottom, LauncherTuning.RPM_MIN);
                double hi = Math.min(Math.max(rpmBottom, rpmTop), LauncherTuning.RPM_MAX);
                rpmTestTarget = clamp(rpmTestTarget - TeleOpDriverDefaults.RPM_TEST_STEP, lo, hi);
                launcher.setTargetRpm(rpmTestTarget);
            } else if (autoSpeedEnabled && !rpmTestEnabled) {
                adjustAutoRpmTweak(-1);
            } else if (!autoSpeedEnabled && manualRpmLocked) {
                adjustManualRpm(-manualRpmStep);
            }
        });
        controls.bindPress(Pad.G1, Btn.DPAD_RIGHT, () -> {
            if (rpmTestEnabled) {
                double lo = Math.max(rpmBottom, LauncherTuning.RPM_MIN);
                double hi = Math.min(Math.max(rpmBottom, rpmTop), LauncherTuning.RPM_MAX);
                rpmTestTarget = clamp(rpmTestTarget + TeleOpDriverDefaults.RPM_TEST_STEP, lo, hi);
                launcher.setTargetRpm(rpmTestTarget);
            } else if (autoSpeedEnabled && !rpmTestEnabled) {
                adjustAutoRpmTweak(+1);
            } else if (!autoSpeedEnabled && manualRpmLocked) {
                adjustManualRpm(manualRpmStep);
            }
        });
        controls.bindPress(Pad.G1, Btn.DPAD_DOWN,  () -> { rpmTestEnabled = false; launcher.stop(); });

        // Right Trigger (manual RPM) — only when AutoSpeed == false, not locked, not test
        controls.bindTriggerAxis(Pad.G1, Trigger.RT, (rt0to1) -> {
            if (!autoSpeedEnabled && !manualRpmLocked && !rpmTestEnabled) {
                double target = rpmBottom + rt0to1 * (rpmTop - rpmBottom);
                launcher.setTargetRpm(target);
            }
        });

        // -------- Gamepad 2 (Co-driver) --------
        controls.bindPress(Pad.G2, Btn.LB, () -> feedOnceWithIntakeAssist());
        controls.bindHold(Pad.G2, Btn.LB, this::markContinuousFireHeld);
        controls.bindPress(Pad.G2, Btn.RB, this::handleIntakeButtonPress);
        controls.bindPress(Pad.G2, Btn.Y,  () -> {
            boolean enable = !autoSpeedEnabled;
            queueAutoSpeedEnablement(enable, /*stopOnDisable=*/false, gamepad1,
                    enable ? AutoSpeedRumble.DOUBLE : AutoSpeedRumble.SINGLE);
        });
        controls.bindPress(Pad.G2, Btn.DPAD_LEFT,  () -> selectVisionProfile(VisionTuning.Mode.P480));
        controls.bindPress(Pad.G2, Btn.DPAD_RIGHT, () -> selectVisionProfile(VisionTuning.Mode.P720));
        controls.bindPress(Pad.G2, Btn.DPAD_UP,    () -> setVisionLiveView(true));
        controls.bindPress(Pad.G2, Btn.DPAD_DOWN,  () -> setVisionLiveView(false));

        // =========================================================================
        // ========================= END CONTROLLER BINDINGS ========================
        // =========================================================================

        // ---- Haptics Init ----
        initAimRumble();

        // ---- Auto RPM Controller Init ----
        ensureAutoCtrl();
        AutoRpmConfig.apply(autoCtrl);      // CENTRALIZED
        autoCtrl.setAutoEnabled(autoSpeedEnabled);

        // ---- FIRST LINE telemetry (init): obelisk memory ----
        List<String> initDashboardLines = new ArrayList<>();
        mirrorData(initDashboardLines, "Obelisk", "%s", ObeliskSignal.getDisplay());
        mirrorData(initDashboardLines, "TeleOp", "Alliance: %s", alliance());
        mirrorData(initDashboardLines, "Startup Defaults", "AutoSpeed=%s  AutoAim=%s  Intake=%s",
                DEFAULT_AUTOSPEED_ENABLED ? "ON" : "OFF",
                DEFAULT_AUTOAIM_ENABLED   ? "ON" : "OFF",
                DEFAULT_INTAKE_ENABLED    ? "ON" : "OFF");
        if (autoStopTimerEnabled) {
            mirrorLine(initDashboardLines, String.format(Locale.US, "⏱ AutoStop: ENABLED (%ds from INIT)", autoStopTimerTimeSec));
        }
        telemetry.update();
        sendDashboard(fusedPose, "INIT", initDashboardLines);
    }

    @Override
    public void init_loop() {
        maybeSeedPoseFromVision();
        if (odometry != null) {
            fusedPose = odometry.getPose();
        }
        List<String> dashboardLines = new ArrayList<>();
        if (feed != null) {
            feed.update();
            if (feed.wasWindowLimitReached()) {
                mirrorLine(dashboardLines, "FeedStop: scale window hit bounds – angles trimmed.");
            } else if (feed.wasAngleClamped()) {
                mirrorLine(dashboardLines, "FeedStop: angles trimmed to fit available span.");
            }
            if (feed.wasSoftLimitClamped() && feed.getSoftLimitMessage() != null) {
                mirrorLine(dashboardLines, feed.getSoftLimitMessage());
            }
            if (feed.wasHomeAborted() && feed.getHomeAbortMessage() != null) {
                mirrorLine(dashboardLines, "FeedStop: " + feed.getHomeAbortMessage());
            }
            mirrorLine(dashboardLines, feed.getFeedStopSummaryLine());
        }
        telemetry.update();
        sendDashboard(fusedPose, "INIT", dashboardLines);
    }

    @Override
    public void start() {
        feed.startFeedStopAfterStart();
        feed.setIdleHoldActive(true);
        intake.set(DEFAULT_INTAKE_ENABLED);
        intakeResumeState = DEFAULT_INTAKE_ENABLED;
        resetIntakeReverseGesture();

        autoAimEnabled = DEFAULT_AUTOAIM_ENABLED;
        aimLossStartMs = -1;
        manualRpmLocked = false;
        rpmTestEnabled = false;
        autoStopTriggered = false;
        stopLatched = false;
        lastStartG1 = false;
        lastStartG2 = false;
        cancelEjectSequence();
        resetIntakeAssistState();
        resetTogglePulseQueue();
        autoSpeedTogglePending = false;
        autoSpeedTogglePad = null;
        autoSpeedToggleRumble = AutoSpeedRumble.NONE;
        autoSpeedToggleStopOnDisable = false;
        autoSpeedToggleTarget = autoSpeedEnabled;

        reverseDriveMode = false;
        longShotMode = false;

        visionProfileSwapInProgress = false;
        visionProfileSwapMode = null;
        visionProfileError = null;

        continuousFireHeld = false;
        continuousFireActive = false;
        continuousFireHoldStartMs = 0L;
        pendingAutoAimNudge = false;
        autoAimNudgeActive = false;
        autoAimNudgeRestoreState = autoAimEnabled;
        resetAutoRpmTweak();

        applyAutoSpeedEnablement(DEFAULT_AUTOSPEED_ENABLED, /*stopOnDisable=*/true);

        teleopInitMillis = System.currentTimeMillis();
    }

    @Override
    public void loop() {
        long now = System.currentTimeMillis();
        if (feed != null) feed.update();
        updateIntakeFlow();
        updatePendingToggleRumbles(now);
        List<String> dashboardLines = new ArrayList<>();

        if (!poseSeeded) {
            maybeSeedPoseFromVision();
        }

        // -------- High-priority Start edge-detect (works even while STOPPED) --------
        boolean start1 = gamepad1.start, start2 = gamepad2.start;
        boolean startPressed = (!lastStartG1 && start1) || (!lastStartG2 && start2);
        lastStartG1 = start1; lastStartG2 = start2;
        if (startPressed) toggleStopLatch(); // toggles STOP ↔ RESUME (calls stopAll() when entering STOP)

        // -------- Optional Auto-Stop timer (top-line telemetry only when enabled) --------
        if (autoStopTimerEnabled) {
            long elapsedMs = Math.max(0, now - teleopInitMillis);
            long remainingMs = Math.max(0, (long) autoStopTimerTimeSec * 1000L - elapsedMs);
            int remSec = (int)Math.ceil(remainingMs / 1000.0);
            int mm = remSec / 60, ss = remSec % 60;

            mirrorLine(dashboardLines, String.format(Locale.US, "⏱ AutoStop: %02d:%02d %s",
                    mm, ss, (autoStopTriggered || stopLatched) ? "(STOPPED)" : ""));

            if (!autoStopTriggered && remainingMs == 0) {
                autoStopTriggered = true;
                stopLatched = true;
                feed.setIdleHoldActive(false); // keep feed fully stopped while StopAll is latched
                feed.setBlock();
                if (intake != null) {
                    intakeResumeState = intake.isOn();
                }
                stopAll();
                mirrorLine(dashboardLines, "⛔ AutoStop reached — STOP ALL engaged (press START to RESUME)");
            }
        }

        // If STOP is latched, hold zero outputs and render minimal status, then return early.
        if (stopLatched) {
            onStoppedLoopHold(dashboardLines);
            sendDashboard(fusedPose, "STOPPED", dashboardLines);
            return;
        }

        // -------- Normal controls (only run when NOT STOPPED) --------
        continuousFireHeld = false; // will be set true by hold bindings during this update

        boolean twistSignTestButton = gamepad1.ps;
        if (twistSignTestButton && !lastTwistSignTestButton) {
            twistSignTestMode = !twistSignTestMode; // Toggle constant-rotation diagnostic
        }
        lastTwistSignTestButton = twistSignTestButton;
        controls.update(gamepad1, gamepad2);
        if (!continuousFireHeld) {
            continuousFireHoldStartMs = 0L;
        }
        drainAutoSpeedQueue();

        now = System.currentTimeMillis();
        updateEjectSequence(now);
        updateIntakeAssist(now);
        boolean ejectActive = isEjectRoutineActive();

        handleContinuousFire(ejectActive);

        // Honor manual lock in manual mode
        if (!autoSpeedEnabled && manualRpmLocked && !rpmTestEnabled && !ejectActive) {
            launcher.setTargetRpm(manualLockedRpm);
        }

        // RPM Test override
        if (rpmTestEnabled) launcher.setTargetRpm(rpmTestTarget);

        // Drive inputs
        double brake = gamepad1.left_trigger;
        double cap = 1.0 - brake * (1.0 - slowestSpeed);

        double driveY  = cap * gamepad1.left_stick_y;
        double strafeX = cap * -gamepad1.left_stick_x;
        double driverRot = cap * -gamepad1.right_stick_x;
        double appliedAimSpeedScale = 1.0;
        double aimRotRaw = Double.NaN;     // Captures aim suggestion pre-sign for telemetry
        double aimRotAfterInvert = Double.NaN;
        boolean aimActive = false;

        boolean anyTagVisible = visionTargetProvider != null && visionTargetProvider.hasAnyTarget();
        boolean goalDetectedRaw = visionTargetProvider != null && visionTargetProvider.isGoalVisibleRaw();
        boolean goalDetectedSmoothed = visionTargetProvider != null && visionTargetProvider.isGoalVisibleSmoothed();
        boolean goalVisibleForAim = visionTargetProvider != null && visionTargetProvider.hasGoalTarget();
        boolean goalVisibleSmoothed = goalDetectedSmoothed;
        double headingDegRaw = visionTargetProvider != null ? visionTargetProvider.getHeadingErrorDeg() : Double.NaN;
        double rangeMetersRaw = visionTargetProvider != null ? visionTargetProvider.getDistanceMeters() : Double.NaN;
        LimelightTargetProvider.DistanceEstimate llDistance = null;
        if (visionTargetProvider instanceof LimelightTargetProvider) {
            llDistance = ((LimelightTargetProvider) visionTargetProvider).getLastDistanceEstimate();
        }
        if (!Double.isNaN(headingDegRaw) && Double.isFinite(headingDegRaw)) {
            smHeadingDeg = (smHeadingDeg == null) ? headingDegRaw : (smoothA * headingDegRaw + (1 - smoothA) * smHeadingDeg);
        }
        if (!Double.isNaN(rangeMetersRaw) && Double.isFinite(rangeMetersRaw)) {
            smRangeMeters = (smRangeMeters == null) ? rangeMetersRaw : (smoothA * rangeMetersRaw + (1 - smoothA) * smRangeMeters);
        }

        updateAutoAimNudge(goalVisibleForAim);
        boolean shotAssistActive = autoAimNudgeActive;

        Double distanceForLockIn = goalVisibleForAim && !Double.isNaN(rangeMetersRaw) && Double.isFinite(rangeMetersRaw)
                ? rangeMetersRaw * M_TO_IN : null;
        if (!AutoAimTuning.LONG_SHOT_ENABLED) {
            longShotMode = false;
        } else if (distanceForLockIn != null) {
            longShotMode = isLongShot(distanceForLockIn);
        }
        LockWindow lockWindow = computeLockWindow(longShotMode, TagAimTuning.DEADBAND_DEG);
        aim.setDeadbandWindow(lockWindow.minDeg, lockWindow.maxDeg);

        // AutoAim + grace handling
        if (autoAimEnabled) {
            appliedAimSpeedScale = clamp(autoAimSpeedScale, 0.0, 1.0);
            driveY  *= appliedAimSpeedScale;
            strafeX *= appliedAimSpeedScale;
            if (goalVisibleSmoothed && goalVisibleForAim) {
                aimLossStartMs = -1L;
                aimRotRaw = aim.turnPower(); // ignore right stick
            } else {
                if (aimLossStartMs < 0) aimLossStartMs = now;
                if ((now - aimLossStartMs) >= autoAimLossGraceMs) {
                    autoAimEnabled = false;
                    aimLossStartMs = -1L;
                    pulseSingle(gamepad1); // disabled after grace
                } else {
                    aimRotRaw = 0.0; // hold heading during grace
                }
            }
            if (autoAimEnabled) {
                aimActive = true;
                aimRotAfterInvert = TagAimController.applyDriveTwistSign(aimRotRaw);
            }
        } else {
            // Manual aim-window rumble when AutoAim is OFF
            if (aimRumbleEnabled && goalVisibleForAim && aimRumbleDriver1 != null) {
                aimRumbleDriver1.update(headingDegRaw);
            }
        }

        if (twistSignTestMode) {
            aimActive = false;
            aimRotRaw = Double.NaN;
            aimRotAfterInvert = Double.NaN;
            driverRot = 0.2; // Positive rotation diagnostic (+ should turn robot clockwise/right)
        }

        if (reverseDriveMode) {
            driveY = -driveY;
            strafeX = -strafeX;
        }

        // Drive it
        double finalRot = aimActive ? aimRotAfterInvert : driverRot;
        drive.drive(driveY, strafeX, finalRot);

        if (odometry != null) {
            fusedPose = odometry.update();
        }

        // AutoSpeed update
        boolean autoRpmActive = (autoSpeedEnabled && !rpmTestEnabled);
        Double autoDistIn = null;
        double autoOutRpm = launcher.targetRpm;
        double autoOutRpmCommanded = launcher.targetRpm;
        Double currentDistanceInches = null;

        if (autoRpmActive && !ejectActive) {
            ensureAutoCtrl();
            AutoRpmConfig.apply(autoCtrl); // CENTRALIZED params + smoothing

            Double rangeM = null;
            if (smRangeMeters != null && Double.isFinite(smRangeMeters)) {
                rangeM = smRangeMeters;
            }

            if (rangeM != null) {
                autoDistIn = rangeM * M_TO_IN;
                autoOutRpm = autoCtrl.updateWithVision(autoDistIn);
                autoHadTagFix = true;
            } else {
                autoOutRpm = (!autoHadTagFix) ? InitialAutoDefaultSpeed : autoCtrl.updateWithVision(null);
            }
            double scaledAutoOut = autoOutRpm * autoRpmTweakFactor;
            autoOutRpmCommanded = clamp(scaledAutoOut, LauncherTuning.RPM_MIN, LauncherTuning.RPM_MAX);
            launcher.setTargetRpm(autoOutRpmCommanded);
        } else if (!ejectActive) {
            // Enforce manual floor if applicable
            if (!manualRpmLocked && !rpmTestEnabled) {
                double currentCmd = launcher.targetRpm;
                if (rpmBottom > 0 && currentCmd < rpmBottom) launcher.setTargetRpm(rpmBottom);
            }
        }

        if (autoDistIn != null) {
            currentDistanceInches = autoDistIn;
        } else if (!Double.isNaN(rangeMetersRaw) && Double.isFinite(rangeMetersRaw)) {
            currentDistanceInches = rangeMetersRaw * M_TO_IN;
        }

        // --- Observe obelisk tags (IDs 21..23) and persist optimal order ---
        if (vision != null) vision.observeObelisk();

        double rpmTarget = getRpmTarget();
        double rpmLeft = getRpmLeft();
        double rpmRight = getRpmRight();
        double rpmAverage = getRpmAverage();

        int bestTagId = (visionTargetProvider != null) ? visionTargetProvider.getBestVisibleTagId() : -1;
        int allianceGoalId = allianceGoalTagId();
        List<Integer> visibleIds = new ArrayList<>();
        LimelightTargetProvider.AimTelemetry aimTelemetry = null;
        if (visionTargetProvider instanceof LimelightTargetProvider) {
            LimelightTargetProvider llProvider = (LimelightTargetProvider) visionTargetProvider;
            allianceGoalId = llProvider.getAllianceGoalId();
            visibleIds = llProvider.getVisibleTagIds();
            aimTelemetry = llProvider.getAimTelemetry();
        } else if (visionTargetProvider != null && visionTargetProvider.hasGoalTarget()) {
            visibleIds.add(allianceGoalId);
        }
        String visibleIdsStr = joinIds(visibleIds);

        List<Integer> obeliskIds = new ArrayList<>();
        for (int id : visibleIds) {
            if (id >= 21 && id <= 23) obeliskIds.add(id);
        }
        boolean obeliskSeen = !obeliskIds.isEmpty();
        String obeliskIdsStr = joinIds(obeliskIds);
        int latchedObeliskId = ObeliskSignal.getLastTagId();
        String latchedObeliskIdStr = (latchedObeliskId < 0) ? "-" : String.valueOf(latchedObeliskId);

        // ---- FIRST LINE telemetry: show obelisk optimal order memory ----
        String obeliskDisplay = ObeliskSignal.getDisplay();
        mirrorData(dashboardLines, "Obelisk", "%s", obeliskDisplay);

        // Telemetry (top block)
        mirrorData(dashboardLines, "Alliance", "%s", alliance());
        mirrorData(dashboardLines, "Intake", intake.getTelemetrySummary());
        mirrorData(dashboardLines, "AutoSpeed", autoSpeedEnabled ? "ON" : "OFF");
        mirrorData(dashboardLines, "AutoAim", autoAimEnabled ? "ON" : "OFF");
        mirrorData(dashboardLines, "Reverse", reverseDriveMode ? "ON" : "OFF");
        String tagVisibleLine;
        if (goalDetectedSmoothed && smHeadingDeg != null && smRangeMeters != null) {
            tagVisibleLine = String.format(Locale.US, "Tag Visible: (#%d, %.1f°, %.0f\")", allianceGoalId, smHeadingDeg, smRangeMeters * M_TO_IN);
        } else {
            tagVisibleLine = "Tag Visible: NO";
        }
        mirrorLine(dashboardLines, tagVisibleLine);
        mirrorData(dashboardLines, "RPM Target / Actual", "%.0f / L:%.0f R:%.0f", rpmTarget, rpmLeft, rpmRight);
        if (autoRpmActive && !rpmTestEnabled && Math.abs(autoRpmTweakFactor - 1.0) > 1e-6) {
            double pct = (autoRpmTweakFactor - 1.0) * 100.0;
            double rpmDelta = autoOutRpmCommanded - autoOutRpm;
            mirrorData(dashboardLines, "AutoRPM Tweak", "ACTIVE: %+4.1f%% (%+.0f rpm)", pct, rpmDelta);
        }
        mirrorData(dashboardLines, "Pose (X,Y,H)", "%.1f, %.1f, %.1f", fusedPose.x, fusedPose.y, fusedPose.headingDeg);

        mirrorLine(dashboardLines, "");

        mirrorData(dashboardLines, "BrakeCap", "%.2f", cap);
        if (manualRpmLocked) mirrorData(dashboardLines, "ManualLock", "LOCKED (%.0f rpm)", manualLockedRpm);
        mirrorData(dashboardLines, "RT", "%.2f", gamepad1.right_trigger);
        if (autoAimEnabled) mirrorData(dashboardLines, "SpeedScale", "%.2f", appliedAimSpeedScale);
        mirrorData(dashboardLines, "Tag Visible (goal)", goalDetectedSmoothed ? "YES" : "NO");
        mirrorData(dashboardLines, "Tag Visible (any)", anyTagVisible ? "YES" : "NO");
        mirrorData(dashboardLines, "Aim Debug", String.format(Locale.US,
                "shotAssist=%s aimActive=%s hasGoalTarget=%s errDeg=%.1f aimRaw=%s aimInv=%s driverRot=%.3f finalRot=%.3f",
                shotAssistActive,
                aimActive,
                goalVisibleForAim,
                (Double.isFinite(headingDegRaw) ? headingDegRaw : Double.NaN),
                Double.isNaN(aimRotRaw) ? "-" : String.format(Locale.US, "%.3f", aimRotRaw),
                Double.isNaN(aimRotAfterInvert) ? "-" : String.format(Locale.US, "%.3f", aimRotAfterInvert),
                driverRot,
                finalRot));
        mirrorData(dashboardLines, "LL: valid/goal/best", String.format(Locale.US,
                "valid=%s anyVisible=%s goalDetected=%s aimValid=%s bestId=%s",
                anyTagVisible,
                anyTagVisible,
                goalDetectedSmoothed,
                goalVisibleForAim,
                (bestTagId < 0) ? "-" : String.valueOf(bestTagId)));
        mirrorData(dashboardLines, "LL: allianceGoalId/ids", String.format(Locale.US,
                "%d / %s",
                allianceGoalId,
                visibleIdsStr));
        if (aimTelemetry != null) {
            String lockedId = (aimTelemetry.lockedAimTagId < 0) ? "-" : String.valueOf(aimTelemetry.lockedAimTagId);
            String aimTxUsed = aimTelemetry.txLockedUsedDeg != null ? String.format(Locale.US, "%.1f", aimTelemetry.txLockedUsedDeg) : "-";
            String lockAgeMs = (aimTelemetry.lockAgeMs < 0) ? "-" : String.valueOf(aimTelemetry.lockAgeMs);
            boolean goalTxFinite = aimTelemetry.goalTxDeg != null && Double.isFinite(aimTelemetry.goalTxDeg);
            String goalTxRaw = aimTelemetry.goalTxDeg == null ? "null" : String.format(Locale.US, "%.2f", aimTelemetry.goalTxDeg);
            mirrorData(dashboardLines, "LL: aimLock", String.format(Locale.US,
                    "detected=%s smoothed=%s aimValid=%s lockFresh=%s locked=%s txUsed=%s goalTx=%s finite=%s ageMs=%s ids=%s",
                    aimTelemetry.goalDetected,
                    aimTelemetry.goalDetectedSmoothed,
                    aimTelemetry.goalAimValid,
                    aimTelemetry.lockFresh,
                    lockedId,
                    aimTxUsed,
                    goalTxRaw,
                    goalTxFinite,
                    lockAgeMs,
                    joinIds(aimTelemetry.visibleIds)));
        }
        mirrorData(dashboardLines, "OB: seen ids/latched", String.format(Locale.US,
                "seen=%s ids=%s latchedId=%s motif=%s ageMs=%d",
                obeliskSeen,
                obeliskIdsStr,
                latchedObeliskIdStr,
                ObeliskSignal.get(),
                ObeliskSignal.ageMs()));
        mirrorData(dashboardLines, "AutoAim Grace (ms)", autoAimLossGraceMs);
        if (autoAimEnabled && aimLossStartMs >= 0 && !goalVisibleSmoothed) {
            mirrorData(dashboardLines, "AutoAim Grace Left", Math.max(0, autoAimLossGraceMs - (now - aimLossStartMs)));
        }

        mirrorData(dashboardLines, "Tag Heading (deg)", (smHeadingDeg == null) ? "---" : String.format(Locale.US, "%.1f", smHeadingDeg));
        Double rawIn = (!Double.isNaN(rangeMetersRaw) && Double.isFinite(rangeMetersRaw)) ? rangeMetersRaw * M_TO_IN : null;
        Double rawTzM = (llDistance != null) ? llDistance.targetForwardMeters : null;
        Double rawTzIn = (rawTzM != null) ? rawTzM * M_TO_IN : null;
        Double fieldIn = (llDistance != null && llDistance.fieldDistanceMeters != null)
                ? llDistance.fieldDistanceMeters * M_TO_IN
                : null;
        updateLimelightTelemetry();
        if (limelightStatusLine != null) mirrorLine(dashboardLines, limelightStatusLine);
        if (limelightHealthLine != null) mirrorLine(dashboardLines, limelightHealthLine);
        updateVisionTelemetry(null, rawIn);
        mirrorData(dashboardLines, "rawTZ_m", (rawTzM == null) ? "---" : String.format(Locale.US, "%.3f", rawTzM));
        mirrorData(dashboardLines, "rawTZ_in", (rawTzIn == null) ? "---" : String.format(Locale.US, "%.1f", rawTzIn));
        mirrorData(dashboardLines, "currentDistanceInches", (currentDistanceInches == null) ? "---" : String.format(Locale.US, "%.1f", currentDistanceInches));
        mirrorData(dashboardLines, "fieldDistanceIn", (fieldIn == null) ? "---" : String.format(Locale.US, "%.1f", fieldIn));
        mirrorData(dashboardLines, "Tag Distance (in)", (rawIn == null) ? "---" : String.format(Locale.US, "%.1f", rawIn));
        mirrorData(dashboardLines, "Tag Dist (in, sm)", (smRangeMeters == null) ? "---" : String.format(Locale.US, "%.1f", smRangeMeters * M_TO_IN));
        mirrorData(dashboardLines, "ShotRangeMode", longShotMode ? "LONG" : "NORMAL");

        if (autoRpmActive && !rpmTestEnabled) {
            mirrorData(dashboardLines, "AutoRPM In (in)", (autoDistIn == null) ? "---" : String.format(Locale.US, "%.1f", autoDistIn));
            mirrorData(dashboardLines, "AutoRPM Out", "%.0f (x%.3f → %.0f)", autoOutRpm, autoRpmTweakFactor, autoOutRpmCommanded);
            double[] calDist = autoCtrl.getCalibrationDistancesIn();
            double[] calRpm  = autoCtrl.getCalibrationSpeedsRpm();
            if (calDist.length > 0) {
                int lastIdx = calDist.length - 1;
                mirrorData(dashboardLines, "AutoRPM Tunables",
                        "%d pts %.0f\"→%.0f … %.0f\"→%.0f",
                        calDist.length,
                        calDist[0], calRpm[0],
                        calDist[lastIdx], calRpm[lastIdx]);
            } else {
                mirrorData(dashboardLines, "AutoRPM Tunables", "No calibration points loaded");
            }
            mirrorData(dashboardLines, "AutoRPM Smoothing α", "%.2f", autoCtrl.getSmoothingAlpha());
        }
        mirrorLine(dashboardLines, visionStatusLine);
        mirrorLine(dashboardLines, visionPerfLine);
        if (visionLightingLine != null) {
            mirrorLine(dashboardLines, visionLightingLine);
        }
        if (visionHealthLine != null) {
            mirrorLine(dashboardLines, visionHealthLine);
        }
        if (visionProfileError != null) {
            mirrorLine(dashboardLines, "Vision profile error: " + visionProfileError);
            visionProfileError = null;
        }
        if (!visionWarningShown && vision != null) {
            String warn = vision.consumeControlWarning();
            if (warn != null) {
                mirrorLine(dashboardLines, warn);
                visionWarningShown = true;
            }
        }
        if (feed != null) {
            if (feed.wasWindowLimitReached()) {
                mirrorLine(dashboardLines, "FeedStop: scale window hit bounds – angles trimmed.");
            } else if (feed.wasAngleClamped()) {
                mirrorLine(dashboardLines, "FeedStop: angles trimmed to fit available span.");
            }
            if (feed.wasSoftLimitClamped() && feed.getSoftLimitMessage() != null) {
                mirrorLine(dashboardLines, feed.getSoftLimitMessage());
            }
            if (feed.wasHomeAborted() && feed.getHomeAbortMessage() != null) {
                mirrorLine(dashboardLines, "FeedStop: " + feed.getHomeAbortMessage());
            }
            mirrorLine(dashboardLines, feed.getFeedStopSummaryLine());
        }
        sendDashboard(fusedPose, "RUN", dashboardLines);
        telemetry.update();
    }

    @Override
    public void stop() {
        // Ensure everything is off when OpMode stops for any reason.
        stopAll();
        try { if (limelight != null) limelight.stop(); } catch (Throwable ignored) {}
        if (visionTaskExecutor != null) {
            visionTaskExecutor.shutdownNow();
            visionTaskExecutor = null;
        }
    }

    // =========================================================================
    // HELPERS
    // =========================================================================
    private double getRpmTarget() {
        return (launcher != null) ? launcher.targetRpm : 0.0;
    }

    private double getRpmLeft() {
        return (launcher != null) ? launcher.getLeftRpm() : 0.0;
    }

    private double getRpmRight() {
        return (launcher != null) ? launcher.getRightRpm() : 0.0;
    }

    private double getRpmAverage() {
        return (launcher != null) ? launcher.getCurrentRpm() : 0.0;
    }

    private void maybeSeedPoseFromVision() {
        if (poseSeeded || odometry == null) return;
        FieldPose guess = odometry.getLastVisionPose();
        if (guess == null && VisionConfig.VISION_SOURCE == VisionSource.LIMELIGHT && limelight != null) {
            odometry.update();
            guess = odometry.getLastVisionPose();
        }
        if (guess != null) {
            fusedPose = guess;
            odometry.setPose(guess.x, guess.y, guess.headingDeg);
            poseSeeded = true;
        }
    }

    private void updateVisionTelemetry(AprilTagDetection aimDet, Double rawDistanceIn) {
        if (vision == null) return;
        long now = System.currentTimeMillis();
        if ((now - lastVisionTelemetryMs) < 100) return; // ~10 Hz updates
        lastVisionTelemetryMs = now;

        if (visionProfileSwapInProgress) {
            String pendingName = "--";
            VisionTuning.Mode mode = visionProfileSwapMode;
            if (mode != null) {
                pendingName = mode.name();
            }
            visionStatusLine = String.format(Locale.US, "Vision: Switching to %s …", pendingName);
            visionPerfLine = "Perf: --- LatMs=--- (profile swap)";
            visionLightingLine = null;
            visionHealthLine = null;
            resetVisionHealthWindow(now);
            return;
        }

        VisionTuning.Profile profile = vision.getActiveProfile();
        if (profile == null) profile = VisionTuning.DEFAULT_PROFILE;

        String profileName = (profile != null && profile.name != null) ? profile.name : "--";
        String liveViewStr = vision.isLiveViewEnabled() ? "ON" : "OFF";
        visionStatusLine = String.format(Locale.US,
                "Vision: Profile=%s LiveView=%s Res=%dx%d@%d Decim=%.1f ProcN=%d MinM=%.0f",
                profileName,
                liveViewStr,
                profile.width,
                profile.height,
                profile.fps,
                profile.decimation,
                profile.processEveryN,
                profile.minDecisionMargin);

        Double fps = vision.getLastKnownFps();
        Double latency = vision.getLastFrameLatencyMs();
        String fpsStr = (fps == null) ? "---" : String.format(Locale.US, "%.1f", fps);
        String latencyStr = (latency == null) ? "---" : String.format(Locale.US, "%.0f", latency);
        visionPerfLine = String.format(Locale.US, "Perf: FPS=%s LatMs=%s", fpsStr, latencyStr);

        VisionAprilTag.BrightnessTelemetry brightness = vision.getBrightnessTelemetry();
        double brightnessMean = Double.NaN;
        if (brightness != null && (brightness.normalizationEnabled || brightness.adaptiveEnabled)) {
            brightnessMean = brightness.smoothedMean;
            String meanStr = Double.isNaN(brightnessMean) ? "---" : String.format(Locale.US, "%.0f", brightnessMean);
            String alphaStr = String.format(Locale.US, "%.2f", brightness.alpha);
            String betaStr = String.format(Locale.US, "%.0f", brightness.beta);
            visionLightingLine = String.format(Locale.US,
                    "VisionLight: Mean=%s→%.0f α=%s β=%s Adaptive=%s",
                    meanStr,
                    brightness.targetMean,
                    alphaStr,
                    betaStr,
                    brightness.adaptiveEnabled ? "ON" : "OFF");
        } else {
            visionLightingLine = null;
        }

        boolean goalVisibleForAim = vision.hasGoodGoalTagForAim(allianceGoalTagId());
        updateVisionHealthTelemetry(goalVisibleForAim, aimDet, brightnessMean, profile, now);
    }

    private void updateLimelightTelemetry() {
        if (VisionConfig.VISION_SOURCE != VisionSource.LIMELIGHT) {
            limelightStatusLine = null;
            limelightHealthLine = null;
            return;
        }

        boolean present = limelight != null;
        boolean resultNull = true;
        boolean resultValid = false;
        Double tx = null;
        boolean botposePresent = false;
        Integer pipelineIdx = null;
        if (limelight != null) {
            LLResult result = null;
            try { result = limelight.getLatestResult(); } catch (Throwable ignored) {}
            resultNull = (result == null);
            if (result != null) {
                resultValid = result.isValid();
                tx = result.getTx();
                Pose3D pose = result.getBotpose_MT2();
                if (pose == null) pose = result.getBotpose();
                botposePresent = pose != null && pose.getPosition() != null;
            }
            pipelineIdx = readPipelineIndex(limelight);
        }

        String pipeStr = (pipelineIdx == null) ? "--" : String.valueOf(pipelineIdx);
        String txStr = (tx == null || Double.isNaN(tx)) ? "--" : String.format(Locale.US, "%.1f", tx);
        limelightStatusLine = String.format(Locale.US,
                "Limelight: present=%s resultNull=%s resultValid=%s pipe=%s tx=%s botpose=%s",
                present, resultNull, resultValid, pipeStr, txStr, botposePresent);

        if (!present) {
            limelightHealthLine = "Limelight not found; ensure EthernetDevice named 'limelight'";
        } else if (resultNull) {
            limelightHealthLine = "Limelight connected; waiting for frames";
        } else {
            limelightHealthLine = null;
        }
    }

    private void applyLimelightPipeline(Limelight3A ll) {
        if (ll == null) return;
        try {
            ll.pipelineSwitch(VisionConfig.LimelightFusion.PIPELINE_INDEX);
            return;
        } catch (Throwable ignored) { }

        try {
            ll.getClass().getMethod("setPipelineIndex", int.class)
                    .invoke(ll, VisionConfig.LimelightFusion.PIPELINE_INDEX);
        } catch (Throwable ignored) { }
    }

    private void applyLimelightPollRate(Limelight3A ll) {
        if (ll == null) return;
        try {
            ll.setPollRateHz(VisionConfig.LimelightFusion.POLL_HZ);
        } catch (Throwable ignored) { }
    }

    private Integer readPipelineIndex(Limelight3A ll) {
        if (ll == null) return null;
        try {
            Object value = ll.getClass().getMethod("getCurrentPipelineIndex").invoke(ll);
            if (value instanceof Number) {
                return ((Number) value).intValue();
            }
        } catch (Throwable ignored) { }
        try {
            Object value = ll.getClass().getMethod("getPipelineIndex").invoke(ll);
            if (value instanceof Number) {
                return ((Number) value).intValue();
            }
        } catch (Throwable ignored) { }
        return null;
    }

    private String joinIds(List<Integer> ids) {
        if (ids == null || ids.isEmpty()) return "-";
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < ids.size(); i++) {
            if (i > 0) sb.append(",");
            sb.append(ids.get(i));
        }
        return sb.toString();
    }

    private void updateVisionHealthTelemetry(boolean goalVisibleForAim,
                                             AprilTagDetection aimDet,
                                             double brightnessMean,
                                             VisionTuning.Profile profile,
                                             long nowMs) {
        if (profile == null) return;
        if (visionHealthWindowStartMs == 0L) visionHealthWindowStartMs = nowMs;

        visionHealthSamples++;
        if (goalVisibleForAim) visionHealthGoodSamples++;
        if (aimDet != null && !Double.isNaN(aimDet.decisionMargin)) {
            visionHealthMarginSum += aimDet.decisionMargin;
            visionHealthMarginCount++;
        }
        if (!Double.isNaN(brightnessMean)) {
            visionHealthBrightnessSum += brightnessMean;
            visionHealthBrightnessCount++;
        }

        if ((nowMs - visionHealthWindowStartMs) < 1500) {
            return;
        }

        double ratio = (visionHealthSamples > 0) ? (double) visionHealthGoodSamples / visionHealthSamples : 0.0;
        double avgMargin = (visionHealthMarginCount > 0) ? visionHealthMarginSum / visionHealthMarginCount : Double.NaN;
        double avgBrightness = (visionHealthBrightnessCount > 0) ? visionHealthBrightnessSum / visionHealthBrightnessCount : Double.NaN;

        boolean brightnessExtreme = !Double.isNaN(avgBrightness)
                && Math.abs(avgBrightness - VisionTuning.TARGET_MEAN_BRIGHTNESS) > VisionTuning.HEALTH_BRIGHTNESS_WARN_DELTA;
        boolean marginLow = !Double.isNaN(avgMargin)
                && avgMargin < (profile.minDecisionMargin - VisionTuning.HEALTH_MARGIN_WARN_DELTA);

        String state;
        if (visionHealthSamples == 0) {
            state = "FAIL";
        } else if (ratio >= VisionTuning.HEALTH_PASS_GOOD_RATIO
                && !marginLow
                && !brightnessExtreme
                && !Double.isNaN(avgMargin)
                && avgMargin >= profile.minDecisionMargin) {
            state = "OK";
        } else if (ratio >= VisionTuning.HEALTH_WARN_GOOD_RATIO && !marginLow && !brightnessExtreme) {
            state = "WEAK";
        } else {
            state = "FAIL";
        }

        String marginStr = Double.isNaN(avgMargin) ? "--" : String.format(Locale.US, "%.1f", avgMargin);
        String brightStr = Double.isNaN(avgBrightness) ? "--" : String.format(Locale.US, "%.0f", avgBrightness);
        visionHealthLine = String.format(Locale.US,
                "Vision Health: %s (good=%d/%d margin=%s min=%.0f mean=%s)",
                state,
                visionHealthGoodSamples,
                visionHealthSamples,
                marginStr,
                profile.minDecisionMargin,
                brightStr);

        resetVisionHealthWindow(nowMs);
    }

    private void resetVisionHealthWindow(long nowMs) {
        visionHealthWindowStartMs = nowMs;
        visionHealthSamples = 0;
        visionHealthGoodSamples = 0;
        visionHealthMarginSum = 0.0;
        visionHealthMarginCount = 0;
        visionHealthBrightnessSum = 0.0;
        visionHealthBrightnessCount = 0;
    }

    private void applyAutoSpeedEnablement(boolean enable, boolean stopOnDisable) {
        ensureAutoCtrl();
        AutoRpmConfig.apply(autoCtrl);

        autoSpeedEnabled = enable;
        autoCtrl.setAutoEnabled(enable);

        if (enable) {
            autoHadTagFix = false;
            int targetId = (alliance() == Alliance.BLUE)
                    ? VisionAprilTag.TAG_BLUE_GOAL
                    : VisionAprilTag.TAG_RED_GOAL;
            AprilTagDetection detNow = (vision != null) ? vision.getDetectionFor(targetId) : null;
            Double seedIn = getGoalDistanceInchesScaled(detNow);

            double seededRpm;
            if (seedIn != null) {
                seededRpm = autoCtrl.updateWithVision(seedIn);
                autoHadTagFix = true;
            } else {
                seededRpm = InitialAutoDefaultSpeed;
            }
            launcher.setTargetRpm(seededRpm);
            manualRpmLocked = false;
        } else {
            autoCtrl.onManualOverride(launcher.getCurrentRpm());
            if (stopOnDisable) {
                launcher.stop();
            }
        }
    }

    private void ensureAutoCtrl() {
        if (autoCtrl == null) autoCtrl = new LauncherAutoSpeedController();
    }

    private void adjustManualRpm(double delta) {
        double lo = Math.max(rpmBottom, LauncherTuning.RPM_MIN);
        double hi = Math.min(Math.max(rpmBottom, rpmTop), LauncherTuning.RPM_MAX);
        double current = manualRpmLocked ? manualLockedRpm : launcher.targetRpm;
        double next = clamp(current + delta, lo, hi);
        if (manualRpmLocked) {
            manualLockedRpm = next;
        }
        launcher.setTargetRpm(next);
    }

    private void toggleReverseDriveMode() {
        reverseDriveMode = !reverseDriveMode;
        if (reverseDriveMode) {
            pulseDouble(gamepad1);
        } else {
            pulseSingle(gamepad1);
        }
    }

    private void selectVisionProfile(VisionTuning.Mode mode) {
        if (vision == null || mode == null) return;
        queueVisionProfileSwap(mode);
        visionWarningShown = false;
        lastVisionTelemetryMs = 0L;
        pulseSingle(gamepad2);
    }

    private void setVisionLiveView(boolean enable) {
        if (vision == null) return;
        vision.toggleLiveView(enable);
        visionWarningShown = false;
        lastVisionTelemetryMs = 0L;
        pulseSingle(gamepad2);
    }

    private void queueAutoSpeedEnablement(boolean enable, boolean stopOnDisable, Gamepad pad,
                                          AutoSpeedRumble rumble) {
        autoSpeedTogglePending = true;
        autoSpeedToggleTarget = enable;
        autoSpeedToggleStopOnDisable = stopOnDisable;
        autoSpeedTogglePad = pad;
        autoSpeedToggleRumble = (rumble != null) ? rumble : AutoSpeedRumble.NONE;
    }

    private void drainAutoSpeedQueue() {
        if (!autoSpeedTogglePending) {
            return;
        }

        boolean enable = autoSpeedToggleTarget;
        boolean stopOnDisable = autoSpeedToggleStopOnDisable;
        Gamepad pad = autoSpeedTogglePad;
        AutoSpeedRumble rumble = autoSpeedToggleRumble;

        autoSpeedTogglePending = false;
        autoSpeedTogglePad = null;
        autoSpeedToggleRumble = AutoSpeedRumble.NONE;

        applyAutoSpeedEnablement(enable, stopOnDisable);

        if (pad != null) {
            if (rumble == AutoSpeedRumble.DOUBLE) {
                pulseDouble(pad);
            } else if (rumble == AutoSpeedRumble.SINGLE) {
                pulseSingle(pad);
            }
        }
    }

    private void queueVisionProfileSwap(VisionTuning.Mode mode) {
        if (vision == null || mode == null) return;

        Runnable task = () -> applyVisionProfileBlocking(mode);

        ExecutorService executor = visionTaskExecutor;
        if (executor == null) {
            task.run();
            return;
        }
        try {
            executor.submit(task);
        } catch (RejectedExecutionException rex) {
            task.run();
        }
    }

    private void applyVisionProfileBlocking(VisionTuning.Mode mode) {
        visionProfileSwapInProgress = true;
        visionProfileSwapMode = mode;
        try {
            vision.applyProfile(mode);
            visionProfileError = null;
        } catch (IllegalStateException ise) {
            String msg = ise.getMessage();
            visionProfileError = (msg != null && !msg.isEmpty()) ? msg : "applyProfile failed";
        } catch (Throwable t) {
            String msg = t.getMessage();
            visionProfileError = (msg != null && !msg.isEmpty()) ? msg : t.getClass().getSimpleName();
        } finally {
            visionProfileSwapInProgress = false;
            visionProfileSwapMode = null;
        }
    }

    private void mirrorData(List<String> mirror, String label, String format, Object... args) {
        telemetry.addData(label, format, args);
        if (mirror != null) {
            mirror.add(label + ": " + String.format(Locale.US, format, args));
        }
    }

    private void mirrorData(List<String> mirror, String label, Object value) {
        mirrorData(mirror, label, "%s", value);
    }

    private void mirrorLine(List<String> mirror, String line) {
        telemetry.addLine(line);
        if (mirror != null) {
            mirror.add(line);
        }
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

    private void initAimRumble() {
        aimRumbleDriver1 = new RumbleNotifier(gamepad1);
        // Aim-window rumble only when AutoAim is OFF → keep enabled here
        aimRumbleDriver1.setActive(true);
        aimRumbleDriver1.setThresholdDeg(aimRumbleDeg);
        aimRumbleDriver1.setMinMax(
                aimRumbleMinStrength, aimRumbleMaxStrength,
                aimRumbleMinPulseMs,  aimRumbleMaxPulseMs,
                aimRumbleMinCooldownMs, aimRumbleMaxCooldownMs
        );
    }

    // --- Rumble helpers (SDK-compatible: no RumbleEffect.builder) ---
    private void pulseDouble(Gamepad gp) {
        gp.rumble((float)togglePulseStrength, (float)togglePulseStrength, togglePulseStepMs);
        int gap = Math.max(0, togglePulseGapMs);
        if (gap <= 0) {
            gp.rumble((float)togglePulseStrength, (float)togglePulseStrength, togglePulseStepMs);
            return;
        }
        long nextAt = System.currentTimeMillis() + gap;
        if (gp == gamepad1) {
            doublePulseQueuedG1 = true;
            doublePulseAtMsG1 = nextAt;
        } else if (gp == gamepad2) {
            doublePulseQueuedG2 = true;
            doublePulseAtMsG2 = nextAt;
        }
    }

    private void pulseSingle(Gamepad gp) {
        gp.rumble((float)togglePulseStrength, (float)togglePulseStrength, togglePulseStepMs + 30);
    }

    private void updatePendingToggleRumbles(long now) {
        if (doublePulseQueuedG1 && now >= doublePulseAtMsG1) {
            gamepad1.rumble((float)togglePulseStrength, (float)togglePulseStrength, togglePulseStepMs);
            doublePulseQueuedG1 = false;
        }
        if (doublePulseQueuedG2 && now >= doublePulseAtMsG2) {
            gamepad2.rumble((float)togglePulseStrength, (float)togglePulseStrength, togglePulseStepMs);
            doublePulseQueuedG2 = false;
        }
    }

    private void resetTogglePulseQueue() {
        doublePulseQueuedG1 = false;
        doublePulseQueuedG2 = false;
        doublePulseAtMsG1 = 0L;
        doublePulseAtMsG2 = 0L;
    }

    private static final class LockWindow {
        final double minDeg;
        final double maxDeg;

        LockWindow(double minDeg, double maxDeg) {
            this.minDeg = minDeg;
            this.maxDeg = maxDeg;
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

    private int allianceGoalTagId() {
        return (alliance() == Alliance.BLUE)
                ? VisionAprilTag.TAG_BLUE_GOAL
                : VisionAprilTag.TAG_RED_GOAL;
    }

    private void adjustAutoRpmTweak(int directionSign) {
        double boundedSign = Math.signum(directionSign);
        if (boundedSign == 0.0) return;
        double candidate = autoRpmTweakFactor * (1.0 + boundedSign * autoRpmTweakScale);
        autoRpmTweakFactor = clamp(candidate, 0.50, 1.50);
    }

    private void resetAutoRpmTweak() {
        autoRpmTweakFactor = 1.0;
    }

    /** Returns SCALED distance to goal in inches if a detection is provided, else null. */
    private Double getGoalDistanceInchesScaled(AprilTagDetection det) {
        if (det == null) return null;
        double rM_sc = vision.getScaledRange(det);
        if (Double.isNaN(rM_sc) || !Double.isFinite(rM_sc)) return null;
        return rM_sc * M_TO_IN;
    }

    /** Records a continuous-fire hold and stamps the start time if this is the first tick. */
    private void markContinuousFireHeld() {
        continuousFireHeld = true;
        if (continuousFireHoldStartMs == 0L) {
            continuousFireHoldStartMs = System.currentTimeMillis();
        }
    }

    /** Flags a request to briefly enable AutoAim before launching a shot. */
    private void requestAutoAimNudge() {
        pendingAutoAimNudge = true;
    }

    /** Restores the driver-selected AutoAim state after a shot assist. */
    private void restoreAutoAimNudgeIfActive() {
        if (!autoAimNudgeActive) {
            return;
        }
        autoAimEnabled = autoAimNudgeRestoreState;
        autoAimNudgeActive = false;
        pendingAutoAimNudge = false;
        aimLossStartMs = -1L;
    }

    /** Enables a temporary AutoAim assist when a tag is visible, then restores the prior setting after firing. */
    private void updateAutoAimNudge(boolean hasGoalTarget) {
        boolean firingActive = feed != null && (feed.isFeedCycleActive() || feed.isContinuousFeedActive());

        if (pendingAutoAimNudge && hasGoalTarget) {
            autoAimNudgeRestoreState = autoAimEnabled;
            autoAimEnabled = true;
            autoAimNudgeActive = true;
            pendingAutoAimNudge = false;
            aimLossStartMs = -1L;
        } else if (pendingAutoAimNudge && !firingActive) {
            pendingAutoAimNudge = false;
        }

        if (autoAimNudgeActive && !firingActive) {
            restoreAutoAimNudgeIfActive();
        }
    }

    /** Feed once, ensuring Intake briefly assists if it was OFF. */
    private void feedOnceWithIntakeAssist() {
        long holdDuration = continuousFireHeld
                ? Math.max(0L, System.currentTimeMillis() - continuousFireHoldStartMs)
                : 0L;
        long holdThreshold = (feed != null) ? feed.getReleaseHoldMs() : 0L;
        boolean holdBlocksSingle = continuousFireHeld && holdThreshold > 0L && holdDuration >= holdThreshold;

        if (ejectPhase != EjectPhase.IDLE || continuousFireActive || holdBlocksSingle) return;
        boolean wasOn = intake.isOn();
        if (feed.beginFeedCycle()) {
            requestAutoAimNudge();
            startIntakeAssist(wasOn, 0L);
        }
    }

    private void handleIntakeButtonPress() {
        if (intake == null) {
            return;
        }

        if (intake.isReversing()) {
            intake.resumeFromReverse();
            resetIntakeReverseGesture();
            return;
        }

        long now = System.currentTimeMillis();
        if (intakeReverseTapCount == 0 || (now - intakeReverseWindowStartMs) > intakeReverseTapWindowMs) {
            intakeReverseTapCount = 0;
            intakeReverseWindowStartMs = now;
            intakeReverseStartState = intake.isOn();
        }

        intakeReverseTapCount++;
        if (intakeReverseTapCount >= 3 && (now - intakeReverseWindowStartMs) <= intakeReverseTapWindowMs) {
            intake.startReverse(intakeReverseStartState);
            resetIntakeReverseGesture();
            return;
        }

        intake.toggle();
    }

    /** Eject one ball asynchronously: spool, feed, hold, then restore previous RPM. */
    private void ejectOnce() {
        if (ejectPhase != EjectPhase.IDLE) return;
        ejectPrevRpmCommand = launcher.targetRpm;
        double tempCmd = clamp(ejectRpm, 0, rpmTop);
        launcher.setTargetRpm(tempCmd);

        startIntakeAssist(intake.isOn(), ejectTimeMs);

        ejectPhase = EjectPhase.SPOOL;
        ejectFeedStarted = false;
        ejectPhaseUntilMs = System.currentTimeMillis() + Math.max(100, ejectTimeMs / 3);
    }

    private void updateEjectSequence(long now) {
        if (ejectPhase == EjectPhase.IDLE) {
            return;
        }

        if (feed == null) {
            cancelEjectSequence();
            return;
        }

        if (ejectPhase == EjectPhase.SPOOL) {
            if (now < ejectPhaseUntilMs) {
                return;
            }
            ejectPhase = EjectPhase.FEED;
        }

        if (ejectPhase == EjectPhase.FEED) {
            if (!ejectFeedStarted) {
                ejectFeedStarted = feed.beginFeedCycle();
            }
            if (!ejectFeedStarted) {
                return; // keep trying until debounce clears
            }
            if (feed.isFeedCycleActive()) {
                return;
            }
            ejectPhase = EjectPhase.HOLD;
            ejectPhaseUntilMs = now + Math.max(0L, ejectTimeMs);
            return;
        }

        if (ejectPhase == EjectPhase.HOLD) {
            if (now < ejectPhaseUntilMs) {
                return;
            }
            launcher.setTargetRpm(ejectPrevRpmCommand);
            ejectPhase = EjectPhase.IDLE;
            ejectFeedStarted = false;
            ejectPhaseUntilMs = 0L;
        }
    }

    private boolean isEjectRoutineActive() {
        return ejectPhase != EjectPhase.IDLE;
    }

    private void updateIntakeAssist(long now) {
        if (!intakeAssistRestorePending) {
            return;
        }

        if (feed == null || intake == null) {
            resetIntakeAssistState();
            return;
        }

        if (intakeAssistWaitingForFeed) {
            if (feed.isFeedCycleActive() || feed.isContinuousFeedActive()) {
                intakeAssistSawFeedActive = true;
                return;
            }
            if (!intakeAssistSawFeedActive) {
                return; // wait until the feed cycle runs once
            }
            intakeAssistWaitingForFeed = false;
            intakeAssistResumeAtMs = now + Math.max(0L, intakeAssistMs + intakeAssistExtraHoldMs);
            return;
        }

        if (intakeAssistResumeAtMs <= 0L || now < intakeAssistResumeAtMs) {
            return;
        }

        boolean current = intake.isOn();
        if (current != intakeAssistTargetState) {
            intake.set(intakeAssistTargetState);
        }
        intakeAssistRestorePending = false;
        intakeAssistResumeAtMs = 0L;
        intakeAssistExtraHoldMs = 0L;
        intakeAssistSawFeedActive = false;
    }

    private void startIntakeAssist(boolean wasOn, long extraHoldMs) {
        if (intake == null) {
            return;
        }
        if (wasOn) {
            return;
        }
        intake.set(true);
        intakeAssistRestorePending = true;
        intakeAssistTargetState = wasOn;
        intakeAssistWaitingForFeed = true;
        intakeAssistSawFeedActive = false;
        intakeAssistResumeAtMs = 0L;
        intakeAssistExtraHoldMs = Math.max(0L, extraHoldMs);
    }

    private void handleContinuousFire(boolean ejectActive) {
        if (feed == null || intake == null) {
            return;
        }

        long holdDuration = continuousFireHeld ? Math.max(0L, System.currentTimeMillis() - continuousFireHoldStartMs) : 0L;
        long holdThreshold = (feed != null) ? feed.getReleaseHoldMs() : 0L;
        boolean holdReady = continuousFireHeld && holdDuration >= holdThreshold;

        if (holdReady && !ejectActive) {
            if (!continuousFireActive) {
                continuousFireActive = true;
                boolean wasOn = intake.isOn();
                feed.startContinuousFeed();
                requestAutoAimNudge();
                startIntakeAssist(wasOn, 0L);
            }
        } else if (continuousFireActive) {
            feed.stopContinuousFeed();
            continuousFireActive = false;
            restoreAutoAimNudgeIfActive();
        }
    }

    private void resetIntakeAssistState() {
        intakeAssistRestorePending = false;
        intakeAssistWaitingForFeed = false;
        intakeAssistTargetState = false;
        intakeAssistResumeAtMs = 0L;
        intakeAssistExtraHoldMs = 0L;
        intakeAssistSawFeedActive = false;
    }

    private void resetIntakeReverseGesture() {
        intakeReverseTapCount = 0;
        intakeReverseWindowStartMs = 0L;
        intakeReverseStartState = (intake != null) && intake.isOn();
    }

    private void cancelEjectSequence() {
        ejectPhase = EjectPhase.IDLE;
        ejectFeedStarted = false;
        ejectPhaseUntilMs = 0L;
    }

    // =========================================================================
    // NEW: StopAll & Latch Mechanics
    // =========================================================================

    /** Immediately stops ALL moving mechanisms and outputs. Safe to call repeatedly. */
    protected void stopAll() {
        cancelEjectSequence();
        resetIntakeAssistState();
        resetIntakeReverseGesture();
        continuousFireHeld = false;
        continuousFireActive = false;
        continuousFireHoldStartMs = 0L;
        if (autoAimNudgeActive) {
            autoAimEnabled = autoAimNudgeRestoreState;
        }
        autoAimNudgeActive = false;
        pendingAutoAimNudge = false;
        resetAutoRpmTweak();

        // DRIVE
        try {
            drive.applyBrakeHold();
        } catch (Throwable t) {
            try { drive.stopAll(); } catch (Throwable ignored) {}
        }

        // LAUNCHER
        try {
            launcher.applyBrakeHold();
        } catch (Throwable t) {
            try { launcher.stop(); } catch (Throwable ignored) {}
        }

        // FEED
        try {
            feed.stopContinuousFeed();
            feed.applyBrakeHold();
        } catch (Throwable t) {
            try { feed.stop(); } catch (Throwable ignored) {}
        }

        // INTAKE
        try {
            intake.applyBrakeHold();
        } catch (Throwable t) {
            try { intake.stop(); } catch (Throwable ignored) {}
        }
    }

    /** Toggle STOP latch. When entering STOP, calls stopAll() and shows telemetry cue; press Start again to resume. */
    private void toggleStopLatch() {
        stopLatched = !stopLatched;
        if (stopLatched) {
            feed.setIdleHoldActive(false); // ensure idle counter-rotation is off while stopped
            feed.setBlock();
            if (intake != null) {
                intakeResumeState = intake.isOn();
            }
            stopAll();
            // Optional haptic cue: single pulse to confirm STOP
            pulseSingle(gamepad1);
        } else {
            feed.setIdleHoldActive(true); // restore idle hold once TeleOp control resumes
            if (intake != null) {
                intake.set(intakeResumeState);
            }
            // Optional haptic cue: single pulse to confirm RESUME
            pulseSingle(gamepad1);
        }
    }

    /** While STOP is latched, continuously enforce 0 outputs and render a concise status line. */
    private void onStoppedLoopHold(List<String> dashboardLines) {
        stopAll(); // defensive: keep everything off each frame
        if (feed != null) feed.update();
        updateIntakeFlow();
        mirrorLine(dashboardLines, "⛔ STOPPED — press START to RESUME");
        telemetry.update();
    }

    private void updateIntakeFlow() {
        if (intake == null) {
            return;
        }
        boolean feedActive = (feed != null) && (feed.isFeedCycleActive() || feed.isContinuousFeedActive());
        intake.update(feedActive);
    }
}
