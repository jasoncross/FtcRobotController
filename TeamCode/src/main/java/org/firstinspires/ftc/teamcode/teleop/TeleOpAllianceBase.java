// ============================================================================
// FILE:           TeleOpAllianceBase.java
// LOCATION:       org/firstinspires/ftc/teamcode/teleop/
// PURPOSE:        Shared TeleOp base (Red/Blue). Manages driver input, drivetrain,
//                 launcher, feed, intake, AprilTag-based aim-assist, and
//                 AUTO LAUNCHER RPM based on AprilTag distance.
//
// STARTUP DEFAULTS (edit here):
//   • DEFAULT_MANUAL_SPEED_MODE = true   → Manual Speed ON at start (AutoRPM OFF)
//   • DEFAULT_AIM_ENABLED       = false  → Auto-Aim OFF at start
//
// HIGHLIGHTS:
//   • Aim-Assist (toggle): preserves translation, controls rotation toward GOAL tag.
//   • Haptics: "aim rumble" window that scales strength/cadence by heading error.
//   • Auto RPM Mode (toggle): when ON and RPM Test Mode is OFF, launcher RPM is
//     computed from AprilTag distance via LauncherAutoSpeedController.
//   • Distance smoothing uses the *scaled* range (meters) from VisionAprilTag so
//     telemetry, auto RPM, and driver info are consistent.
//   • AutoRPM telemetry is SHOWN ONLY when Manual Speed is OFF (AutoRPM ON).
//   • NEW: Manual RPM Lock (Square/X) in manual mode to freeze current RPM.
//
// CONTROLS (Gamepad 1):
//   Left stick .......... Fwd/Back + Strafe
//   Right stick X ....... Rotation
//   Left trigger ........ Brake (caps speed toward slowestSpeed)
//   Right trigger ....... Manual launch RPM (only when manual & not locked & not test)
//   LB .................. Feed one ball
//   RB .................. Toggle intake ON/OFF
//   Right Stick Button .. Toggle Aim-Assist  [double-rumble]
//   Y (Triangle) ........ Toggle Manual/Auto RPM mode  [double-rumble]
//   X (Square) .......... Toggle Manual RPM LOCK (manual mode only)  [double-rumble]
//   D-pad Up ............ Enable RPM TEST MODE
//   D-pad Left/Right .... -/+ 50 RPM while TEST MODE enabled (applies immediately)
//   D-pad Down .......... Disable RPM TEST MODE and STOP launcher
//
// TELEMETRY:
//   Always: Alliance, BrakeCap, Intake, ManualSpeed, ManualLock, RT, RPM Target/Actual,
//           Aim Enabled, Tag Visible, Heading (deg), Tag Distance (in), Tag Distance (in, sm)
//   When AutoRPM active (ManualSpeed OFF and Test OFF): AutoRPM In/Out, Tunables, Smoothing, Last
//
// NOTES:
//   - Aim rumble policy: ONLY active when Aim-Assist is OFF.
//   - Auto RPM: linear mapping with extrapolation; holds last RPM if tag is lost.
//   - Uses *scaled* meters from VisionAprilTag for both smoothing and telemetry.
// AUTHOR:         Indianola Robotics – 2025 Season (DECODE)
// LAST UPDATED:   2025-10-22
// ============================================================================
package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad; // for Gamepad.RumbleEffect
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.drive.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Feed;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;

// === VISION IMPORTS ===
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.vision.VisionAprilTag;
import org.firstinspires.ftc.teamcode.vision.TagAimController;

import java.util.ArrayList;
import java.util.List;

// === CONTROLLER BINDINGS ===
import org.firstinspires.ftc.teamcode.input.ControllerBindings;
import static org.firstinspires.ftc.teamcode.input.ControllerBindings.Pad;
import static org.firstinspires.ftc.teamcode.input.ControllerBindings.Btn;
import static org.firstinspires.ftc.teamcode.input.ControllerBindings.Trigger;

// === HAPTICS (RUMBLE) ===
import org.firstinspires.ftc.teamcode.util.RumbleNotifier;

// === AUTO LAUNCHER SPEED (RPM) ===
import org.firstinspires.ftc.teamcode.control.LauncherAutoSpeedController;

public abstract class TeleOpAllianceBase extends OpMode {
    protected abstract Alliance alliance();

    // ---------------- Startup Defaults (change here) ----------------
    private static final boolean DEFAULT_MANUAL_SPEED_MODE = true;  // Manual ON → AutoRPM OFF
    private static final boolean DEFAULT_AIM_ENABLED       = false; // Auto-Aim OFF

    // ---------------- Subsystems ----------------
    protected Drivebase drive;
    protected Launcher launcher;
    protected Feed feed;
    protected Intake intake;

    // ---------------- Tunables ----------------
    private double slowestSpeed = 0.25;
    private double rpmBottom    = 0;
    private double rpmTop       = 6000;

    // ---------------- State ----------------
    private boolean manualSpeedMode = DEFAULT_MANUAL_SPEED_MODE;  // toggled by Y
    private boolean aimEnabled      = DEFAULT_AIM_ENABLED;        // toggled by Right Stick Button

    // Manual RPM Lock (Square/X)
    private boolean manualRpmLocked = false;
    private double  manualLockedRpm = 0.0;

    // ---------------- Vision + Aim ----------------
    private VisionAprilTag vision;
    private TagAimController aim = new TagAimController();

    // ---------------- Pose/Range Smoothing (SCALED meters) ----------------
    private Double smHeadingDeg = null;
    private Double smRangeMeters = null;
    private static final double SMOOTH_A = 0.25;
    private static final double M_TO_IN = 39.37007874015748;

    // ---------------- Controller Bindings ----------------
    private ControllerBindings controls;

    // ---------------- RPM Test Mode ----------------
    private boolean rpmTestEnabled = false;
    private double  rpmTestTarget  = 0.0;
    private static final double RPM_TEST_STEP = 50.0;
    private static final double RPM_TEST_MIN  = 0.0;
    private static final double RPM_TEST_MAX  = 6000.0;

    // ---------------- Aim Rumble (Haptics) ----------------
    private RumbleNotifier aimRumbleDriver1;
    private boolean aimRumbleEnabled       = true;   // master enable/disable
    private double  aimRumbleDeg           = 2.5;

    private double  aimRumbleMinStrength   = 0.10;
    private double  aimRumbleMaxStrength   = 0.65;

    private int     aimRumbleMinPulseMs    = 120;
    private int     aimRumbleMaxPulseMs    = 200;

    private int     aimRumbleMinCooldownMs = 120;
    private int     aimRumbleMaxCooldownMs = 350;

    // ---------------- Toggle Pulse Settings ----------------
    private double togglePulseStrength     = 0.8;
    private int    togglePulseStepMs       = 120;
    private int    togglePulseGapMs        = 80;

    // ---------------- Auto Launcher Speed (RPM) ----------------
    private LauncherAutoSpeedController autoCtrl;

    // INITIAL TUNABLES for Auto RPM (update in code as you tune)
    private double autoNearDistIn = 24.0;
    private double autoNearRpm    = 1000.0;
    private double autoFarDistIn  = 120.0;
    private double autoFarRpm     = 4500.0;

    // Optional smoothing 0..1 (0 = off). Try 0.10–0.30 for gentle smoothing.
    private double autoSmoothingAlpha = 0.15;

    // Local clamp helper (keeps RPM in range)
    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    @Override
    public void init() {
        // ---- Subsystem Initialization ----
        drive    = new Drivebase(hardwareMap, telemetry);
        launcher = new Launcher(hardwareMap);
        feed     = new Feed(hardwareMap);
        intake   = new Intake(hardwareMap);

        // ---- Vision Initialization ----
        vision = new VisionAprilTag();
        vision.init(hardwareMap, "Webcam 1");
        vision.setRangeScale(0.03);

        // ---- Controller Bindings Setup ----
        controls = new ControllerBindings();

        controls
            .bindPress(Pad.G1, Btn.LB, () -> feed.feedOnceBlocking())
            .bindPress(Pad.G1, Btn.RB, () -> intake.toggle())
            .bindToggle(Pad.G1, Btn.R_STICK_BTN,
                () -> { aimEnabled = true;  pulseDoubleToggle(gamepad1); },
                () -> { aimEnabled = false; pulseDoubleToggle(gamepad1); })
            .bindToggle(Pad.G1, Btn.Y,
                // -> MANUAL ON
                () -> {
                    manualSpeedMode = true;
                    ensureAutoCtrl();
                    autoCtrl.onManualOverride(launcher.getCurrentRpm());
                    autoCtrl.setAutoEnabled(false);
                    pulseDoubleToggle(gamepad1);
                },
                // -> AUTO ON
                () -> {
                    manualSpeedMode = false;
                    ensureAutoCtrl();
                    autoCtrl.setAutoEnabled(true);

                    // Seed with current *scaled* distance (inches) so we don't hold an old RPM
                    int targetId = (alliance() == Alliance.BLUE)
                            ? VisionAprilTag.TAG_BLUE_GOAL
                            : VisionAprilTag.TAG_RED_GOAL;
                    AprilTagDetection detNow = vision.getDetectionFor(targetId);
                    Double seedIn = getGoalDistanceInchesScaled(detNow);
                    double seededRpm = autoCtrl.updateWithVision(seedIn);
                    launcher.setTargetRpm(seededRpm);

                    pulseDoubleToggle(gamepad1);
                })
            // Manual RPM set from RT — ONLY when manual, not locked, and not in test
            .bindTriggerAxis(Pad.G1, Trigger.RT, (rt0to1) -> {
                if (manualSpeedMode && !manualRpmLocked && !rpmTestEnabled) {
                    double target = rpmBottom + rt0to1 * (rpmTop - rpmBottom);
                    launcher.setTargetRpm(target);
                }
            })
            // RPM TEST MODE
            .bindPress(Pad.G1, Btn.DPAD_UP, () -> {
                rpmTestEnabled = true;
                launcher.setTargetRpm(rpmTestTarget);
            })
            .bindPress(Pad.G1, Btn.DPAD_LEFT, () -> {
                if (rpmTestEnabled) {
                    rpmTestTarget = clamp(rpmTestTarget - RPM_TEST_STEP, RPM_TEST_MIN, RPM_TEST_MAX);
                    launcher.setTargetRpm(rpmTestTarget);
                }
            })
            .bindPress(Pad.G1, Btn.DPAD_RIGHT, () -> {
                if (rpmTestEnabled) {
                    rpmTestTarget = clamp(rpmTestTarget + RPM_TEST_STEP, RPM_TEST_MIN, RPM_TEST_MAX);
                    launcher.setTargetRpm(rpmTestTarget);
                }
            })
            .bindPress(Pad.G1, Btn.DPAD_DOWN, () -> {
                rpmTestEnabled = false;
                launcher.stop();
            })
            // === NEW: Manual RPM LOCK (Square/X) toggle ===
            .bindToggle(Pad.G1, Btn.X,
                // -> LOCK ON
                () -> {
                    if (manualSpeedMode) {
                        manualRpmLocked = true;
                        manualLockedRpm = launcher.targetRpm; // lock what we’re currently commanding
                        launcher.setTargetRpm(manualLockedRpm); // ensure it’s applied
                        pulseDoubleToggle(gamepad1);
                    }
                },
                // -> LOCK OFF
                () -> {
                    manualRpmLocked = false;
                    pulseDoubleToggle(gamepad1);
                });

        // Co-driver mirrors key toggles (no joysticks)
        controls
            .bindPress(Pad.G2, Btn.LB, () -> feed.feedOnceBlocking())
            .bindPress(Pad.G2, Btn.RB, () -> intake.toggle())
            .bindToggle(Pad.G2, Btn.Y,
                () -> {
                    manualSpeedMode = true;
                    ensureAutoCtrl();
                    autoCtrl.onManualOverride(launcher.getCurrentRpm());
                    autoCtrl.setAutoEnabled(false);
                    pulseDoubleToggle(gamepad1);
                },
                () -> {
                    manualSpeedMode = false;
                    ensureAutoCtrl();
                    autoCtrl.setAutoEnabled(true);

                    int targetId = (alliance() == Alliance.BLUE)
                            ? VisionAprilTag.TAG_BLUE_GOAL
                            : VisionAprilTag.TAG_RED_GOAL;
                    AprilTagDetection detNow = vision.getDetectionFor(targetId);
                    Double seedIn = getGoalDistanceInchesScaled(detNow);
                    double seededRpm = autoCtrl.updateWithVision(seedIn);
                    launcher.setTargetRpm(seededRpm);

                    pulseDoubleToggle(gamepad1);
                });

        // ---- Haptics Init ----
        initAimRumble();

        // ---- Auto RPM Controller Init ----
        ensureAutoCtrl(); // obeys DEFAULT_MANUAL_SPEED_MODE

        telemetry.addData("TeleOp", "Alliance: %s", alliance());
        telemetry.addData("Startup Defaults", "ManualSpeed=%s  Aim=%s",
                DEFAULT_MANUAL_SPEED_MODE ? "ON" : "OFF",
                DEFAULT_AIM_ENABLED ? "ON" : "OFF");
        telemetry.update();
    }

    @Override
    public void loop() {
        // ==========================
        // UPDATE CONTROLLER BINDINGS
        // ==========================
        controls.update(gamepad1, gamepad2);

        // If manual lock is on, continuously hold the locked RPM in manual mode
        if (manualSpeedMode && manualRpmLocked && !rpmTestEnabled) {
            launcher.setTargetRpm(manualLockedRpm);
        }

        // RPM Test Mode override (wins over manual/auto this loop)
        if (rpmTestEnabled) {
            launcher.setTargetRpm(rpmTestTarget);
        }

        // ==========================
        // DRIVETRAIN CONTROL
        // ==========================
        double brake = gamepad1.left_trigger;
        double cap = 1.0 - brake * (1.0 - slowestSpeed);

        double driveY  = cap * gamepad1.left_stick_y;
        double strafeX = cap * -gamepad1.left_stick_x;
        double twist   = cap * -gamepad1.right_stick_x;

        // ==========================
        // ALLIANCE TARGET TAG
        // ==========================
        int targetId = (alliance() == Alliance.BLUE)
                ? VisionAprilTag.TAG_BLUE_GOAL
                : VisionAprilTag.TAG_RED_GOAL;

        // ==============================================================
        // APRILTAG DETECTION + SMOOTHING (use *SCALED* meters) + AIM OVERRIDE
        // ==============================================================
        AprilTagDetection goalDet = vision.getDetectionFor(targetId);

        if (goalDet != null) {
            double hDeg  = goalDet.ftcPose.bearing;        // degrees
            double rM_sc = vision.getScaledRange(goalDet); // SCALED meters (NaN if invalid)

            // Smooth heading
            smHeadingDeg = (smHeadingDeg == null) ? hDeg
                    : (SMOOTH_A * hDeg + (1 - SMOOTH_A) * smHeadingDeg);

            // Smooth *scaled* range if valid
            if (!Double.isNaN(rM_sc) && Double.isFinite(rM_sc)) {
                smRangeMeters = (smRangeMeters == null) ? rM_sc
                        : (SMOOTH_A * rM_sc + (1 - SMOOTH_A) * smRangeMeters);
            }
        }

        if (aimEnabled && goalDet != null) {
            twist = aim.turnPower(goalDet);
        }

        // ==========================
        // DRIVEBASE EXECUTION
        // ==========================
        drive.drive(driveY, strafeX, twist);

        // ==============================================================
        // AUTO RPM UPDATE (only when ManualSpeed OFF AND Test OFF)
        //  - Uses smoothed *scaled* meters → inches (preferred), else current frame.
        //  - Holds last RPM when tag not visible or invalid.
        // ==============================================================
        boolean autoRpmActive = (!manualSpeedMode && !rpmTestEnabled);
        Double autoDistIn = null;
        double autoOutRpm = launcher.targetRpm; // for telemetry

        if (autoRpmActive) {
            ensureAutoCtrl();

            // Prefer smoothed scaled meters if available
            if (smRangeMeters != null && Double.isFinite(smRangeMeters)) {
                autoDistIn = smRangeMeters * M_TO_IN;
            } else {
                // Fallback: current frame scaled meters
                double mNow = vision.getScaledRange(goalDet); // NaN if no tag/invalid
                if (!Double.isNaN(mNow) && Double.isFinite(mNow)) {
                    autoDistIn = mNow * M_TO_IN;
                }
            }

            autoOutRpm = autoCtrl.updateWithVision(autoDistIn);
            launcher.setTargetRpm(autoOutRpm);
        }

        // ==========================
        // TELEMETRY
        // ==========================
        telemetry.addData("Alliance", alliance());
        telemetry.addData("BrakeCap", "%.2f", cap);
        telemetry.addData("Intake", intake.isOn() ? "On" : "Off");
        telemetry.addData("ManualSpeed", manualSpeedMode);
        telemetry.addData("ManualLock", manualRpmLocked ? String.format("LOCKED @ %.0f", manualLockedRpm) : "UNLOCKED");
        telemetry.addData("RT", "%.2f", gamepad1.right_trigger);
        telemetry.addData("RPM Target", "%.0f", launcher.targetRpm);
        telemetry.addData("RPM Actual", "%.0f", launcher.getCurrentRpm());

        telemetry.addData("Aim Enabled", aimEnabled);
        telemetry.addData("Target Tag ID", targetId);

        List<Integer> seenIds = new ArrayList<>();
        for (AprilTagDetection d : vision.getDetectionsCompat()) seenIds.add(d.id);
        telemetry.addData("Seen Tags (count)", seenIds.size());
        telemetry.addData("Seen Tag IDs", seenIds.isEmpty() ? "[]"
                : seenIds.toString());

        double headingDeg = (smHeadingDeg == null) ? Double.NaN : smHeadingDeg;

        // Distances — both from *scaled* meters for consistency
        double m_direct_sc    = vision.getScaledRange(goalDet); // scaled meters (NaN if none)
        double in_direct_sc   = Double.isNaN(m_direct_sc) ? Double.NaN : m_direct_sc * M_TO_IN;
        double in_smoothed_sc = (smRangeMeters == null) ? Double.NaN : smRangeMeters * M_TO_IN;

        boolean tagVisible = (goalDet != null);

        telemetry.addData("Goal Tag Visible", tagVisible);
        telemetry.addData("Goal Heading (deg)", Double.isNaN(headingDeg) ? "---" : String.format("%.1f", headingDeg));
        telemetry.addData("Tag Distance (in)",    Double.isNaN(in_direct_sc)   ? "---" : String.format("%.1f", in_direct_sc));
        telemetry.addData("Tag Distance (in, sm)",Double.isNaN(in_smoothed_sc) ? "---" : String.format("%.1f", in_smoothed_sc));

        // ---- AutoRPM telemetry (gated) ----
        if (autoRpmActive) {
            telemetry.addData("AutoRPM In (in)", autoDistIn == null ? "---" : String.format("%.1f", autoDistIn));
            telemetry.addData("AutoRPM Out", "%.0f rpm", autoOutRpm);
            telemetry.addData("AutoRPM Tunables", "Near: %.0f in @ %.0f rpm | Far: %.0f in @ %.0f rpm",
                    autoNearDistIn, autoNearRpm, autoFarDistIn, autoFarRpm);
            telemetry.addData("AutoRPM Smoothing", "%.2f", autoSmoothingAlpha);
            telemetry.addData("AutoRPM Last", "%.0f", (autoCtrl != null ? autoCtrl.getLastAutoRpm() : 0.0));
        }

        // Aim rumble status + update
        boolean allowRumble = aimRumbleEnabled && !aimEnabled;
        telemetry.addData("Aim Rumble", allowRumble ? "ENABLED (Aim OFF)" :
                (aimRumbleEnabled ? "DISABLED (Aim ON)" : "DISABLED (Switch)"));

        if (allowRumble) {
            updateAimRumbleWith(headingDeg, tagVisible);
        }

        // RPM Test telemetry only when enabled
        if (rpmTestEnabled) telemetry.addData("RPM Test", "ENABLED");
        if (rpmTestEnabled) telemetry.addData("RPM Test Target", "%.0f", rpmTestTarget);

        telemetry.update();
    }

    @Override
    public void stop() {
        if (vision != null) vision.stop();
    }

    // ====================================================================================================
    //  HAPTICS (RUMBLE) HELPERS
    // ====================================================================================================
    private void initAimRumble() {
        aimRumbleDriver1 = new RumbleNotifier(gamepad1);
        aimRumbleDriver1.setThresholdDeg(aimRumbleDeg);
        aimRumbleDriver1.setStrengthRange(aimRumbleMinStrength, aimRumbleMaxStrength);
        aimRumbleDriver1.setPulseRange(aimRumbleMinPulseMs, aimRumbleMaxPulseMs);
        aimRumbleDriver1.setCooldownRange(aimRumbleMinCooldownMs, aimRumbleMaxCooldownMs);
    }

    private void updateAimRumbleWith(double yawErrorDeg, boolean tagVisible) {
        if (aimRumbleDriver1 == null) return;

        // Keep helper in sync with any runtime tweaks
        aimRumbleDriver1.setThresholdDeg(aimRumbleDeg);
        aimRumbleDriver1.setStrengthRange(aimRumbleMinStrength, aimRumbleMaxStrength);
        aimRumbleDriver1.setPulseRange(aimRumbleMinPulseMs, aimRumbleMaxPulseMs);
        aimRumbleDriver1.setCooldownRange(aimRumbleMinCooldownMs, aimRumbleMaxCooldownMs);

        aimRumbleDriver1.update(yawErrorDeg, tagVisible);

        telemetry.addData("Rumble Window (±deg)", aimRumbleDeg);
        telemetry.addData("Rumble Scale", String.format("S[%.2f..%.2f] P[%d..%d] Cd[%d..%d]",
                aimRumbleMinStrength, aimRumbleMaxStrength,
                aimRumbleMinPulseMs, aimRumbleMaxPulseMs,
                aimRumbleMinCooldownMs, aimRumbleMaxCooldownMs));
    }

    /** Plays a crisp double-pulse on the given gamepad for state toggles (Aim/ManualSpeed/Lock). */
    private void pulseDoubleToggle(Gamepad pad) {
        try {
            Gamepad.RumbleEffect effect = new Gamepad.RumbleEffect.Builder()
                    .addStep(togglePulseStrength, togglePulseStrength, togglePulseStepMs) // pulse 1
                    .addStep(0, 0, togglePulseGapMs)                                     // gap
                    .addStep(togglePulseStrength, togglePulseStrength, togglePulseStepMs) // pulse 2
                    .build();
            pad.runRumbleEffect(effect);
        } catch (Throwable t) {
            pad.rumble(togglePulseStrength, togglePulseStrength, togglePulseStepMs);
        }
    }

    // ====================================================================================================
    //  AUTO RPM CONTROLLER HELPERS
    // ====================================================================================================
    private void ensureAutoCtrl() {
        if (autoCtrl == null) {
            autoCtrl = new LauncherAutoSpeedController();
            autoCtrl.setParams(autoNearDistIn, autoNearRpm, autoFarDistIn, autoFarRpm);
            autoCtrl.setSmoothingAlpha(autoSmoothingAlpha);
            autoCtrl.setAutoEnabled(!manualSpeedMode);
        } else {
            // Keep enabled state synced with the current manual/auto mode
            autoCtrl.setAutoEnabled(!manualSpeedMode);
        }
    }

    /** Returns best-available GOAL distance in INCHES from *scaled* meters. */
    private Double getGoalDistanceInchesScaled(AprilTagDetection det) {
        if (det != null) {
            double mDirect = vision.getScaledRange(det); // NaN if invalid
            if (!Double.isNaN(mDirect) && Double.isFinite(mDirect)) {
                return mDirect * M_TO_IN;
            }
        }
        if (smRangeMeters != null && Double.isFinite(smRangeMeters)) {
            return smRangeMeters * M_TO_IN;
        }
        return null; // no usable distance
    }
}
