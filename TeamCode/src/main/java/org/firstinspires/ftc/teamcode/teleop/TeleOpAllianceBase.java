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
 */
package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.drive.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Feed;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;

// === VISION IMPORTS ===
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.vision.VisionAprilTag;
import org.firstinspires.ftc.teamcode.vision.TagAimController;
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
import org.firstinspires.ftc.teamcode.config.TeleOpDriverDefaults; // TeleOp-only workflow + manual ranges
import org.firstinspires.ftc.teamcode.config.TeleOpEjectTuning;    // TeleOp-only eject routine
import org.firstinspires.ftc.teamcode.config.TeleOpRumbleTuning;   // Driver rumble envelopes
import org.firstinspires.ftc.teamcode.config.VisionTuning;         // AprilTag range calibration

import java.util.Locale;

public abstract class TeleOpAllianceBase extends OpMode {
    // CHANGES (2025-10-30): Added AutoAim drive speed scaling, manual RPM D-pad nudges (AutoSpeed off & lock engaged), and telemetry updates.
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
    private boolean autoAimEnabled   = DEFAULT_AUTOAIM_ENABLED;   // Live state for AprilTag aim assist

    // Manual RPM Lock (Square/X) — only when AutoSpeed == false
    private boolean manualRpmLocked = false; // Manual RPM hold toggle (Square/X) when AutoSpeed disabled
    private double  manualLockedRpm = 0.0;   // Stored RPM when manualRpmLocked is true

    // ---------------- Vision + Aim ----------------
    private VisionAprilTag vision;                // Shared AprilTag pipeline for aim + autospeed
    private TagAimController aim = new TagAimController(); // PD twist helper; TeleOp clamps via SharedRobotTuning
    private double autoAimSpeedScale = AutoAimTuning.AUTO_AIM_SPEED_SCALE; // Translation multiplier while AutoAim is active

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

    // ---------------- RPM Test Mode ----------------
    private boolean rpmTestEnabled = false; // Manual RPM sweep test (D-pad adjustments)
    private double  rpmTestTarget  = 0.0;   // Current manual test RPM when enabled

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

    // ---------------- NEW: StopAll / Latch & Auto-Stop Timer ----------------
    /** When true, all outputs are forced to zero every loop until Start is pressed again. */
    private boolean stopLatched = false; // When true, StopAll has latched and outputs remain zeroed

    /** Auto-Stop timer master enable (defaults false). When true, shows top-line countdown and calls StopAll at 0. */
    protected boolean autoStopTimerEnabled = TeleOpDriverDefaults.AUTO_STOP_TIMER_ENABLED; // Optional endgame auto-stop timer flag

    /** Auto-Stop timer seconds from TeleOp INIT (defaults 119). */
    protected int autoStopTimerTimeSec = TeleOpDriverDefaults.AUTO_STOP_TIMER_TIME_SEC; // Default countdown seconds when timer enabled

    /** Timestamp captured at TeleOp INIT; used as the timer start. */
    private long teleopInitMillis = 0L; // TeleOp init timestamp for timer calculations

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
        intake.set(DEFAULT_INTAKE_ENABLED);

        // ---- Vision Initialization ----
        vision = new VisionAprilTag();
        vision.init(hardwareMap, "Webcam 1");
        vision.setRangeScale(VisionTuning.RANGE_SCALE); // keep your calibration scale unless re-tuned

        // ---- Controller Bindings Setup ----
        controls = new ControllerBindings();

        // =========================================================================
        // ========== CONTROLLER BINDINGS: ALL MAPPINGS CENTRALIZED (EDIT HERE) ====
        // =========================================================================

        // -------- Gamepad 1 (Driver) --------
        // Feed / Intake
        controls.bindPress(Pad.G1, Btn.LB, () -> feedOnceWithIntakeAssist());
        controls.bindPress(Pad.G1, Btn.RB, () -> intake.toggle());

        // AutoAim toggle (gated by current tag visibility)
        controls.bindPress(Pad.G1, Btn.R_STICK_BTN, () -> {
            int targetId = (alliance() == Alliance.BLUE)
                    ? VisionAprilTag.TAG_BLUE_GOAL
                    : VisionAprilTag.TAG_RED_GOAL;
            AprilTagDetection detNow = vision.getDetectionFor(targetId);

            if (!autoAimEnabled) {
                if (detNow != null) {
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
            autoSpeedEnabled = !autoSpeedEnabled;
            ensureAutoCtrl();
            AutoRpmConfig.apply(autoCtrl); // CENTRALIZED: used to be local tunables here
            autoCtrl.setAutoEnabled(autoSpeedEnabled);

            if (autoSpeedEnabled) {
                autoHadTagFix = false;
                int targetId = (alliance() == Alliance.BLUE)
                        ? VisionAprilTag.TAG_BLUE_GOAL
                        : VisionAprilTag.TAG_RED_GOAL;
                AprilTagDetection detNow = vision.getDetectionFor(targetId);
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
                pulseDouble(gamepad1);
            } else {
                autoCtrl.onManualOverride(launcher.getCurrentRpm());
                pulseSingle(gamepad1);
            }
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
        controls.bindPress(Pad.G2, Btn.RB, () -> intake.toggle());
        controls.bindPress(Pad.G2, Btn.Y,  () -> {
            autoSpeedEnabled = !autoSpeedEnabled;
            ensureAutoCtrl();
            AutoRpmConfig.apply(autoCtrl); // CENTRALIZED
            autoCtrl.setAutoEnabled(autoSpeedEnabled);

            if (autoSpeedEnabled) {
                autoHadTagFix = false;
                int targetId = (alliance() == Alliance.BLUE)
                        ? VisionAprilTag.TAG_BLUE_GOAL
                        : VisionAprilTag.TAG_RED_GOAL;
                AprilTagDetection detNow = vision.getDetectionFor(targetId);
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
                pulseDouble(gamepad1);
            } else {
                autoCtrl.onManualOverride(launcher.getCurrentRpm());
                pulseSingle(gamepad1);
            }
        });

        // =========================================================================
        // ========================= END CONTROLLER BINDINGS ========================
        // =========================================================================

        // ---- Haptics Init ----
        initAimRumble();

        // ---- Auto RPM Controller Init ----
        ensureAutoCtrl();
        AutoRpmConfig.apply(autoCtrl);      // CENTRALIZED
        autoCtrl.setAutoEnabled(autoSpeedEnabled);

        // ---- Auto-Stop Timer: base timestamp at TeleOp INIT ----
        teleopInitMillis = System.currentTimeMillis();
        autoStopTriggered = false;
        stopLatched = false; // start un-stopped

        // ---- FIRST LINE telemetry (init): obelisk memory ----
        telemetry.addData("Obelisk", ObeliskSignal.getDisplay());
        telemetry.addData("TeleOp", "Alliance: %s", alliance());
        telemetry.addData("Startup Defaults", "AutoSpeed=%s  AutoAim=%s  Intake=%s",
                DEFAULT_AUTOSPEED_ENABLED ? "ON" : "OFF",
                DEFAULT_AUTOAIM_ENABLED   ? "ON" : "OFF",
                DEFAULT_INTAKE_ENABLED    ? "ON" : "OFF");
        if (autoStopTimerEnabled) {
            telemetry.addLine(String.format(Locale.US, "⏱ AutoStop: ENABLED (%ds from INIT)", autoStopTimerTimeSec));
        }
        telemetry.update();
    }

    @Override
    public void loop() {
        // -------- High-priority Start edge-detect (works even while STOPPED) --------
        boolean start1 = gamepad1.start, start2 = gamepad2.start;
        boolean startPressed = (!lastStartG1 && start1) || (!lastStartG2 && start2);
        lastStartG1 = start1; lastStartG2 = start2;
        if (startPressed) toggleStopLatch(); // toggles STOP ↔ RESUME (calls stopAll() when entering STOP)

        // -------- Optional Auto-Stop timer (top-line telemetry only when enabled) --------
        if (autoStopTimerEnabled) {
            long now = System.currentTimeMillis();
            long elapsedMs = Math.max(0, now - teleopInitMillis);
            long remainingMs = Math.max(0, (long) autoStopTimerTimeSec * 1000L - elapsedMs);
            int remSec = (int)Math.ceil(remainingMs / 1000.0);
            int mm = remSec / 60, ss = remSec % 60;

            telemetry.addLine(String.format(Locale.US, "⏱ AutoStop: %02d:%02d %s",
                    mm, ss, (autoStopTriggered || stopLatched) ? "(STOPPED)" : ""));

            if (!autoStopTriggered && remainingMs == 0) {
                autoStopTriggered = true;
                stopLatched = true;
                stopAll();
                telemetry.addLine("⛔ AutoStop reached — STOP ALL engaged (press START to RESUME)");
            }
        }

        // If STOP is latched, hold zero outputs and render minimal status, then return early.
        if (stopLatched) {
            onStoppedLoopHold();
            return;
        }

        // -------- Normal controls (only run when NOT STOPPED) --------
        controls.update(gamepad1, gamepad2);

        // Honor manual lock in manual mode
        if (!autoSpeedEnabled && manualRpmLocked && !rpmTestEnabled) {
            launcher.setTargetRpm(manualLockedRpm);
        }

        // RPM Test override
        if (rpmTestEnabled) launcher.setTargetRpm(rpmTestTarget);

        // Drive inputs
        double brake = gamepad1.left_trigger;
        double cap = 1.0 - brake * (1.0 - slowestSpeed);

        double driveY  = cap * gamepad1.left_stick_y;
        double strafeX = cap * -gamepad1.left_stick_x;
        double twist   = cap * -gamepad1.right_stick_x;
        double appliedAimSpeedScale = 1.0;

        // Alliance target
        int targetId = (alliance() == Alliance.BLUE)
                ? VisionAprilTag.TAG_BLUE_GOAL
                : VisionAprilTag.TAG_RED_GOAL;

        // Vision
        AprilTagDetection goalDet = vision.getDetectionFor(targetId);

        if (goalDet != null) {
            double hDeg  = goalDet.ftcPose.bearing;
            double rM_sc = vision.getScaledRange(goalDet);
            smHeadingDeg = (smHeadingDeg == null) ? hDeg : (smoothA * hDeg + (1 - smoothA) * smHeadingDeg);
            if (!Double.isNaN(rM_sc) && Double.isFinite(rM_sc)) {
                smRangeMeters = (smRangeMeters == null) ? rM_sc : (smoothA * rM_sc + (1 - smoothA) * smRangeMeters);
            }
        }

        // AutoAim + grace handling
        long now = System.currentTimeMillis();
        if (autoAimEnabled) {
            appliedAimSpeedScale = clamp(autoAimSpeedScale, 0.0, 1.0);
            driveY  *= appliedAimSpeedScale;
            strafeX *= appliedAimSpeedScale;
            if (goalDet != null) {
                aimLossStartMs = -1L;
                twist = aim.turnPower(goalDet); // ignore right stick
            } else {
                if (aimLossStartMs < 0) aimLossStartMs = now;
                if ((now - aimLossStartMs) >= autoAimLossGraceMs) {
                    autoAimEnabled = false;
                    aimLossStartMs = -1L;
                    pulseSingle(gamepad1); // disabled after grace
                } else {
                    twist = 0.0; // hold heading during grace
                }
            }
        } else {
            // Manual aim-window rumble when AutoAim is OFF
            if (aimRumbleEnabled && goalDet != null && aimRumbleDriver1 != null) {
                aimRumbleDriver1.update(goalDet.ftcPose.bearing);
            }
        }

        // Drive it
        drive.drive(driveY, strafeX, twist);

        // AutoSpeed update
        boolean autoRpmActive = (autoSpeedEnabled && !rpmTestEnabled);
        Double autoDistIn = null;
        double autoOutRpm = launcher.targetRpm;

        if (autoRpmActive) {
            ensureAutoCtrl();
            AutoRpmConfig.apply(autoCtrl); // CENTRALIZED params + smoothing

            Double rangeM = null;
            if (smRangeMeters != null && Double.isFinite(smRangeMeters)) {
                rangeM = smRangeMeters;
            } else if (goalDet != null) {
                double rM_sc = vision.getScaledRange(goalDet);
                if (!Double.isNaN(rM_sc) && Double.isFinite(rM_sc)) rangeM = rM_sc;
            }

            if (rangeM != null) {
                autoDistIn = rangeM * M_TO_IN;
                autoOutRpm = autoCtrl.updateWithVision(autoDistIn);
                autoHadTagFix = true;
            } else {
                autoOutRpm = (!autoHadTagFix) ? InitialAutoDefaultSpeed : autoCtrl.updateWithVision(null);
            }
            launcher.setTargetRpm(autoOutRpm);
        } else {
            // Enforce manual floor if applicable
            if (!manualRpmLocked && !rpmTestEnabled) {
                double currentCmd = launcher.targetRpm;
                if (rpmBottom > 0 && currentCmd < rpmBottom) launcher.setTargetRpm(rpmBottom);
            }
        }

        // --- Observe obelisk tags (IDs 21..23) and persist optimal order ---
        if (vision != null) vision.observeObelisk();

        // ---- FIRST LINE telemetry: show obelisk optimal order memory ----
        telemetry.addData("Obelisk", ObeliskSignal.getDisplay());

        // Telemetry
        telemetry.addData("Alliance", "%s", alliance());
        telemetry.addData("BrakeCap", "%.2f", cap);
        telemetry.addData("Intake", intake.isOn() ? "ON" : "OFF");
        telemetry.addData("AutoSpeed", autoSpeedEnabled ? "ON" : "OFF");
        if (manualRpmLocked) telemetry.addData("ManualLock", "LOCKED (%.0f rpm)", manualLockedRpm);
        telemetry.addData("RT", "%.2f", gamepad1.right_trigger);
        telemetry.addData("RPM Target/Actual", "%.0f / %.0f", launcher.targetRpm, launcher.getCurrentRpm());

        telemetry.addData("AutoAim", autoAimEnabled ? "ON" : "OFF");
        if (autoAimEnabled) telemetry.addData("SpeedScale", String.format(Locale.US, "%.2f", appliedAimSpeedScale));
        telemetry.addData("Tag Visible", (goalDet != null) ? "YES" : "NO");
        telemetry.addData("AutoAim Grace (ms)", autoAimLossGraceMs);
        if (autoAimEnabled && aimLossStartMs >= 0 && goalDet == null) {
            telemetry.addData("AutoAim Grace Left", Math.max(0, autoAimLossGraceMs - (now - aimLossStartMs)));
        }

        telemetry.addData("Tag Heading (deg)", (smHeadingDeg == null) ? "---" : String.format("%.1f", smHeadingDeg));
        Double rawIn = getGoalDistanceInchesScaled(goalDet);
        telemetry.addData("Tag Distance (in)", (rawIn == null) ? "---" : String.format("%.1f", rawIn));
        telemetry.addData("Tag Dist (in, sm)", (smRangeMeters == null) ? "---" : String.format("%.1f", smRangeMeters * M_TO_IN));

        if (autoRpmActive && !rpmTestEnabled) {
            telemetry.addData("AutoRPM In (in)", (autoDistIn == null) ? "---" : String.format("%.1f", autoDistIn));
            telemetry.addData("AutoRPM Out", "%.0f", autoOutRpm);
            telemetry.addData("AutoRPM Tunables", "Near=%.0f→%.0f  Far=%.0f→%.0f",
                    autoCtrl.getNearDistanceIn(), autoCtrl.getNearSpeedRpm(),
                    autoCtrl.getFarDistanceIn(),  autoCtrl.getFarSpeedRpm());
            telemetry.addData("AutoRPM Smoothing α", "%.2f", autoCtrl.getSmoothingAlpha());
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        // Ensure everything is off when OpMode stops for any reason.
        stopAll();
    }

    // =========================================================================
    // HELPERS
    // =========================================================================
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
        sleepMs(togglePulseGapMs);
        gp.rumble((float)togglePulseStrength, (float)togglePulseStrength, togglePulseStepMs);
    }

    private void pulseSingle(Gamepad gp) {
        gp.rumble((float)togglePulseStrength, (float)togglePulseStrength, togglePulseStepMs + 30);
    }

    /** Returns SCALED distance to goal in inches if a detection is provided, else null. */
    private Double getGoalDistanceInchesScaled(AprilTagDetection det) {
        if (det == null) return null;
        double rM_sc = vision.getScaledRange(det);
        if (Double.isNaN(rM_sc) || !Double.isFinite(rM_sc)) return null;
        return rM_sc * M_TO_IN;
    }

    /** Feed once, ensuring Intake briefly assists if it was OFF. */
    private void feedOnceWithIntakeAssist() {
        boolean wasOn = intake.isOn();
        if (!wasOn) intake.set(true);
        feed.feedOnceBlocking();
        if (!wasOn) {
            sleepMs(intakeAssistMs);
            intake.set(false);
        }
    }

    /** Eject one ball: temporarily set launcher to EjectRPM, feed once w/ Intake Assist, then restore previous RPM. */
    private void ejectOnce() {
        double prevCmd = launcher.targetRpm;
        double tempCmd = clamp(ejectRpm, 0, rpmTop);

        launcher.setTargetRpm(tempCmd);
        sleepMs(Math.max(100, ejectTimeMs / 3));

        boolean wasOn = intake.isOn();
        if (!wasOn) intake.set(true);

        feed.feedOnceBlocking();

        sleepMs(ejectTimeMs);

        if (!wasOn) {
            sleepMs(intakeAssistMs);
            intake.set(false);
        }

        launcher.setTargetRpm(prevCmd); // restore exact prior commanded RPM
    }

    private void sleepMs(int ms) {
        try { Thread.sleep(ms); } catch (InterruptedException ignored) {}
    }

    // =========================================================================
    // NEW: StopAll & Latch Mechanics
    // =========================================================================

    /** Immediately stops ALL moving mechanisms and outputs. Safe to call repeatedly. */
    protected void stopAll() {
        // DRIVE
        try {
            // Prefer explicit stop() if your Drivebase exposes it
            drive.stop();
        } catch (Throwable t) {
            try { drive.drive(0, 0, 0); } catch (Throwable ignored) {}
        }

        // LAUNCHER
        try {
            launcher.stop();
        } catch (Throwable t) {
            try { launcher.setTargetRpm(0); } catch (Throwable ignored) {}
        }

        // FEED
        try {
            feed.stop();
        } catch (Throwable t) {
            try { feed.setPower(0); } catch (Throwable ignored) {}
        }

        // INTAKE
        try {
            intake.stop();
        } catch (Throwable t) {
            try { intake.set(false); } catch (Throwable ignored) {}
        }
    }

    /** Toggle STOP latch. When entering STOP, calls stopAll() and shows telemetry cue; press Start again to resume. */
    private void toggleStopLatch() {
        stopLatched = !stopLatched;
        if (stopLatched) {
            stopAll();
            // Optional haptic cue: single pulse to confirm STOP
            pulseSingle(gamepad1);
        } else {
            // Optional haptic cue: single pulse to confirm RESUME
            pulseSingle(gamepad1);
        }
    }

    /** While STOP is latched, continuously enforce 0 outputs and render a concise status line. */
    private void onStoppedLoopHold() {
        stopAll(); // defensive: keep everything off each frame
        telemetry.addLine("⛔ STOPPED — press START to RESUME");
        telemetry.update();
    }
}
