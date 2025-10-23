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

/*
 * FILE: TeleOpAllianceBase.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleop/
 *
 * PURPOSE:
 *   Shared TeleOp base for both Red and Blue alliances.
 *   Manages driver input, drivetrain, launcher, feed, intake, and AprilTag-based auto-aim.
 *   Aim-Assist keeps the robot pointed at the alliance GOAL tag while preserving full translation control.
 *   OPTIONAL: Haptic "aim rumble" tells the driver when robot heading is within a small window of the tag,
 *             scaled from softer at the window edge to stronger and faster near center (0° error).
 *
 * CONTROLS (Gamepad 1):
 *   Left stick ........ Forward/back + strafe
 *   Right stick X ..... Rotation
 *   Left trigger ...... Brake (reduces top speed toward slowestSpeed)
 *   Right trigger ..... Manual launch RPM (only when manualSpeedMode == true)
 *   Left Bumper ....... Feed one ball (Feed subsystem)
 *   Right Bumper ...... Toggle intake ON/OFF
 *   Right Stick Btn ... TOGGLE Aim-Assist (not hold)  [double-rumble on toggle]
 *   Triangle / Y ...... Toggle manual launch-speed mode [double-rumble on toggle]
 *   D-pad Up .......... Enable RPM TEST MODE
 *   D-pad Left/Right .. -/+ 50 RPM while TEST MODE enabled (applies immediately)
 *   D-pad Down ........ Disable RPM TEST MODE and STOP launcher
 *
 * TELEMETRY (always shown):
 *   Alliance, BrakeCap, Intake state, ManualSpeed, RT value,
 *   RPM Target/Actual, Aim Enabled, Tag Visible, Tag Heading (deg),
 *   Tag Distance (inches), Aim Rumble status.
 *
 * TELEMETRY (only when enabled):
 *   RPM Test, RPM Test Target.
 *
 * NOTES:
 *   - See ControllerBindings.java header for authoritative binding list.
 *   - Gamepad rumble requires a controller with haptics (e.g., Xbox/PS). Some pads (e.g., Logitech F310) do not rumble.
 *   - Aim rumble policy: ONLY active when Aim-Assist is OFF (manual aiming).
 *   - Auto RPM: when manualSpeedMode == false and RPM Test Mode is OFF, launcher RPM is computed from AprilTag distance
 *               using LauncherAutoSpeedController (linear with extrapolation + hold-last if tag is lost).
 */

public abstract class TeleOpAllianceBase extends OpMode {
    protected abstract Alliance alliance();

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
    private boolean manualSpeedMode = true;   // TOGGLED (Y) — double-pulse on change
    private boolean aimEnabled = false;       // TOGGLED (R-Click) — double-pulse on change

    // ---------------- Vision + Aim ----------------
    private VisionAprilTag vision;
    private TagAimController aim = new TagAimController();

    // ---------------- Pose Smoothing ----------------
    private Double smHeadingDeg = null;
    private Double smRangeMeters = null;
    private static final double SMOOTH_A = 0.25;
    private static final double M_TO_IN = 39.37007874015748;

    // ---------------- Controller Bindings ----------------
    private ControllerBindings controls;

    // ---------------- RPM Test Mode ----------------
    // PURPOSE: Allow bench-testing launcher RPM via D-pad (Up/Down/Left/Right).
    private boolean rpmTestEnabled = false;
    private double  rpmTestTarget  = 0.0;

    private static final double RPM_TEST_STEP = 50.0;   // increment per Left/Right press
    private static final double RPM_TEST_MIN  = 0.0;
    private static final double RPM_TEST_MAX  = 6000.0;

    // ---------------- Aim Rumble (Haptics) ----------------
    // PURPOSE: Tactile feedback when heading to the selected AprilTag is within ±aimRumbleDeg.
    // POLICY:  ONLY rumble when Aim-Assist is OFF (aimEnabled == false).
    // CONTROL: Master switch aimRumbleEnabled allows disabling haptics.
    private RumbleNotifier aimRumbleDriver1;
    private boolean aimRumbleEnabled       = true;   // master enable/disable
    private double  aimRumbleDeg           = 2.5;    // ±degrees window for "on target"

    // Intensity scales edge→center
    private double  aimRumbleMinStrength   = 0.10;   // intensity at the window edge
    private double  aimRumbleMaxStrength   = 0.65;   // intensity at 0° error

    // Pulse duration scales edge→center
    private int     aimRumbleMinPulseMs    = 120;    // shorter at edge
    private int     aimRumbleMaxPulseMs    = 200;    // longer near center

    // Cooldown scales center→edge (shorter near center → faster pulses)
    private int     aimRumbleMinCooldownMs = 120;    // fastest cadence near center
    private int     aimRumbleMaxCooldownMs = 350;    // slowest cadence at edge

    // ---------------- Toggle Pulse Settings ----------------
    // PURPOSE: Double-pulse the controller on key state changes for clarity to the driver.
    private double togglePulseStrength     = 0.8;    // double-pulse intensity for toggles
    private int    togglePulseStepMs       = 120;    // each step duration
    private int    togglePulseGapMs        = 80;     // gap between pulses (handled by effect)

    // ---------------- Auto Launcher Speed (RPM) ----------------
    // PURPOSE: Compute launcher target RPM from AprilTag distance when manualSpeedMode == false.
    private LauncherAutoSpeedController autoCtrl;

    // INITIAL TUNABLES (team defaults; adjust via code or FTC Dashboard later)
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

        /*
         * GAMEPAD 1 (Driver)
         *   LB  -> Feed once
         *   RB  -> Intake toggle
         *   RS  -> Aim-assist toggle (double-pulse on toggle)
         *   Y   -> Manual-speed mode toggle (double-pulse on toggle)
         *   RT  -> Manual RPM set (axis) [G1 ONLY]
         *   D-PAD -> RPM Test Mode controls (Up/Left/Right/Down)
         */
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
                    // Seed auto controller with current launcher RPM for seamless return to auto.
                    ensureAutoCtrl(); // make sure it's constructed before we access it
                    autoCtrl.onManualOverride(launcher.getCurrentRpm());
                    autoCtrl.setAutoEnabled(false);
                    pulseDoubleToggle(gamepad1);
                },
                // -> AUTO ON
                () -> {
                    manualSpeedMode = false;
                    ensureAutoCtrl();
                    autoCtrl.setAutoEnabled(true);
                    pulseDoubleToggle(gamepad1);
                })
            .bindTriggerAxis(Pad.G1, Trigger.RT, (rt0to1) -> {
                if (manualSpeedMode && !rpmTestEnabled) {
                    double target = rpmBottom + rt0to1 * (rpmTop - rpmBottom);
                    launcher.setTargetRpm(target);
                }
            })

            // --- RPM TEST MODE (D-PAD) kept in the same chain ---
            // Up: enable test mode and apply current target immediately
            .bindPress(Pad.G1, Btn.DPAD_UP, () -> {
                rpmTestEnabled = true;
                launcher.setTargetRpm(rpmTestTarget); // immediate apply helps convergence
            })
            // Left/Right: adjust target by ±50 (clamped) and apply immediately
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
            // Down: disable test mode and stop launcher
            .bindPress(Pad.G1, Btn.DPAD_DOWN, () -> {
                rpmTestEnabled = false;
                launcher.stop();
            });

        /*
         * GAMEPAD 2 (Co-Driver) — Same functions, EXCEPT anything involving joysticks:
         *   LB -> Feed once
         *   RB -> Intake toggle
         *   Y  -> Manual-speed mode toggle (mirrors state; feedback is on G1 only)
         */
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
                    pulseDoubleToggle(gamepad1);
                });

        // ---- Haptics Init ----
        initAimRumble();

        // ---- Auto RPM Controller Init ----
        ensureAutoCtrl(); // builds and configures with current defaults

        telemetry.addData("TeleOp", "Alliance: %s", alliance());
        telemetry.addLine("Bindings: See ControllerBindings.java for bound functions.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // ==============================================================
        // UPDATE CONTROLLER BINDINGS FIRST
        // ==============================================================
        controls.update(gamepad1, gamepad2);

        // ==============================================================
        // RPM TEST MODE OVERRIDE (wins over manual/auto in this loop)
        // ==============================================================
        if (rpmTestEnabled) {
            launcher.setTargetRpm(rpmTestTarget);
        }

        // ==============================================================
        // DRIVETRAIN CONTROL
        // ==============================================================
        double brake = gamepad1.left_trigger;
        double cap = 1.0 - brake * (1.0 - slowestSpeed);

        double driveY  = cap * gamepad1.left_stick_y;
        double strafeX = cap * -gamepad1.left_stick_x;
        double twist   = cap * -gamepad1.right_stick_x;

        // ==============================================================
        // ALLIANCE TARGET TAG SELECTION
        // ==============================================================
        int targetId = (alliance() == Alliance.BLUE)
                ? VisionAprilTag.TAG_BLUE_GOAL
                : VisionAprilTag.TAG_RED_GOAL;

        // ==============================================================
        // APRILTAG DETECTION + SMOOTHING + AIM OVERRIDE
        // ==============================================================
        AprilTagDetection goalDet = vision.getDetectionFor(targetId);

        if (goalDet != null) {
            double h = goalDet.ftcPose.bearing;
            double r = goalDet.ftcPose.range;
            smHeadingDeg  = (smHeadingDeg  == null) ? h : (SMOOTH_A * h + (1 - SMOOTH_A) * smHeadingDeg);
            smRangeMeters = (smRangeMeters == null) ? r : (SMOOTH_A * r + (1 - SMOOTH_A) * smRangeMeters);
        }

        if (aimEnabled && goalDet != null) {
            twist = aim.turnPower(goalDet);
        }

        // ==============================================================
        // DRIVEBASE EXECUTION
        // ==============================================================
        drive.drive(driveY, strafeX, twist);

        // ==============================================================
        // AUTO RPM UPDATE (applies only when manualSpeedMode == false AND test mode is OFF)
        // - Uses smoothed distance when available (inches).
        // - Holds last RPM when tag not visible.
        // ==============================================================
        if (!manualSpeedMode && !rpmTestEnabled) {
            ensureAutoCtrl();
            Double distIn = null;
            if (smRangeMeters != null) {
                double d = smRangeMeters * M_TO_IN;
                if (!Double.isNaN(d) && Double.isFinite(d)) distIn = d;
            }
            double targetRpm = autoCtrl.updateWithVision(distIn);
            launcher.setTargetRpm(targetRpm);
        }

        // ==============================================================
        // TELEMETRY OUTPUT (original + vision + auto RPM)
        // ==============================================================
        telemetry.addData("Alliance", alliance());
        telemetry.addData("BrakeCap", "%.2f", cap);
        telemetry.addData("Intake", intake.isOn() ? "On" : "Off");
        telemetry.addData("ManualSpeed", manualSpeedMode);
        telemetry.addData("RT", "%.2f", gamepad1.right_trigger);
        telemetry.addData("RPM Target", "%.0f", launcher.targetRpm);
        telemetry.addData("RPM Actual", "%.0f", launcher.getCurrentRpm());

        telemetry.addData("Aim Enabled", aimEnabled);
        telemetry.addData("Target Tag ID", targetId);

        List<Integer> seenIds = new ArrayList<>();
        for (AprilTagDetection d : vision.getDetectionsCompat()) seenIds.add(d.id);
        telemetry.addData("Seen Tags (count)", seenIds.size());
        telemetry.addData("Seen Tag IDs", seenIds.isEmpty() ? "[]" : seenIds.toString());

        double headingDeg = (smHeadingDeg == null) ? Double.NaN : smHeadingDeg;

        // Prefer smoothed range for RPM stability; still show a direct sample for transparency
        double distM_direct = vision.getScaledRange(goalDet);
        double distIn_direct = Double.isNaN(distM_direct) ? Double.NaN : distM_direct * 39.3701;
        double distIn_smoothed = (smRangeMeters == null) ? Double.NaN : smRangeMeters * 39.3701;

        boolean tagVisible = (goalDet != null);

        telemetry.addData("Goal Tag Visible", tagVisible);
        telemetry.addData("Goal Heading (deg)", Double.isNaN(headingDeg) ? "---" : String.format("%.1f", headingDeg));
        telemetry.addData("Tag Distance (in)", Double.isNaN(distIn_direct) ? "---" : String.format("%.1f (direct)", distIn_direct));
        telemetry.addData("Tag Distance (in, sm)", Double.isNaN(distIn_smoothed) ? "---" : String.format("%.1f (sm)", distIn_smoothed));

        // Auto RPM telemetry
        telemetry.addData("AutoRPM Enabled", (!manualSpeedMode && !rpmTestEnabled));
        telemetry.addData("AutoRPM Tunables", "Near: %.0f in @ %.0f rpm | Far: %.0f in @ %.0f rpm",
                autoNearDistIn, autoNearRpm, autoFarDistIn, autoFarRpm);
        telemetry.addData("AutoRPM Smoothing", "%.2f", autoSmoothingAlpha);
        telemetry.addData("AutoRPM Last", "%.0f", (autoCtrl != null ? autoCtrl.getLastAutoRpm() : 0.0));

        // ==============================================================
        // AIM RUMBLE UPDATE (HAPTICS)
        // POLICY: Only rumble when Aim-Assist is OFF (aimEnabled == false) and master switch is ON.
        // PURPOSE: Give subtle aim feedback during manual aiming; silence when auto-aim is controlling twist.
        // ==============================================================
        boolean allowRumble = aimRumbleEnabled && !aimEnabled;
        telemetry.addData("Aim Rumble", allowRumble ? "ENABLED (Aim OFF)" :
                (aimRumbleEnabled ? "DISABLED (Aim ON)" : "DISABLED (Switch)"));

        if (allowRumble) {
            updateAimRumbleWith(headingDeg, tagVisible);
        }

        // Show RPM Test telemetry only when enabled
        if (rpmTestEnabled) telemetry.addData("RPM Test", "ENABLED");
        if (rpmTestEnabled) telemetry.addData("RPM Test Target", "%.0f", rpmTestTarget);

        telemetry.update();
    }

    @Override
    public void stop() {
        if (vision != null) vision.stop();
    }

    // ====================================================================================================
    //  SECTION:       HAPTICS (RUMBLE) HELPERS
    //  PURPOSE:       Initialization, per-loop aim rumble update, and toggle double-pulse.
    //  CONFIG:        aimRumbleEnabled (master), aimRumbleDeg (± degrees),
    //                 StrengthRange[min..max], PulseRange[min..max], CooldownRange[min..max].
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

        // Keep helper in sync with any runtime tweaks (cheap, safe)
        aimRumbleDriver1.setThresholdDeg(aimRumbleDeg);
        aimRumbleDriver1.setStrengthRange(aimRumbleMinStrength, aimRumbleMaxStrength);
        aimRumbleDriver1.setPulseRange(aimRumbleMinPulseMs, aimRumbleMaxPulseMs);
        aimRumbleDriver1.setCooldownRange(aimRumbleMinCooldownMs, aimRumbleMaxCooldownMs);

        // Use smoothed heading (deg) and current visibility.
        aimRumbleDriver1.update(yawErrorDeg, tagVisible);

        telemetry.addData("Rumble Window (±deg)", aimRumbleDeg);
        telemetry.addData("Rumble Scale", String.format("S[%.2f..%.2f] P[%d..%d] Cd[%d..%d]",
                aimRumbleMinStrength, aimRumbleMaxStrength,
                aimRumbleMinPulseMs, aimRumbleMaxPulseMs,
                aimRumbleMinCooldownMs, aimRumbleMaxCooldownMs));
    }

    /** Plays a crisp double-pulse on the given gamepad for state toggles (Aim/ManualSpeed). */
    private void pulseDoubleToggle(Gamepad pad) {
        try {
            Gamepad.RumbleEffect effect = new Gamepad.RumbleEffect.Builder()
                    .addStep(togglePulseStrength, togglePulseStrength, togglePulseStepMs) // pulse 1
                    .addStep(0, 0, togglePulseGapMs)                                     // gap
                    .addStep(togglePulseStrength, togglePulseStrength, togglePulseStepMs) // pulse 2
                    .build();
            pad.runRumbleEffect(effect);
        } catch (Throwable t) {
            // Fallback: single pulse if effect not supported (older SDK/controllers)
            pad.rumble(togglePulseStrength, togglePulseStrength, togglePulseStepMs);
        }
    }

    // ====================================================================================================
    //  SECTION:       AUTO RPM CONTROLLER HELPERS
    //  PURPOSE:       Construction + parameterization in one place.
    // ====================================================================================================
    private void ensureAutoCtrl() {
        if (autoCtrl == null) {
            autoCtrl = new LauncherAutoSpeedController();
            autoCtrl.setParams(autoNearDistIn, autoNearRpm, autoFarDistIn, autoFarRpm);
            autoCtrl.setSmoothingAlpha(autoSmoothingAlpha);
            autoCtrl.setAutoEnabled(!manualSpeedMode);
        }
    }
}
