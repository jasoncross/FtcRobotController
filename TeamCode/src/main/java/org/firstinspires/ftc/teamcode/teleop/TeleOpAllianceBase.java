// ============================================================================
// FILE:           TeleOpAllianceBase.java
// LOCATION:       TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleop/
//
// PURPOSE:
//   Shared TeleOp base for both Red and Blue alliances.
//   Manages driver input, drivetrain, launcher, feed, intake, and AprilTag-based
//   Aim-Assist. Also computes AUTO LAUNCHER RPM (“AutoSpeed”) from AprilTag
//   distance using a tunable mapping.
//
//   Aim-Assist keeps the robot pointed at the alliance GOAL tag while preserving
//   full translation control. Haptics give the driver tactile feedback on
//   toggles and while manually aiming (“aim window” rumble).
//
//   AutoSpeed continuously computes launcher RPM from tag distance. If the tag
//   isn’t visible, the launcher holds the most recent auto RPM. When AutoSpeed
//   is first enabled with no visible tag, the launcher runs at InitialAutoDefaultSpeed
//   until a tag is seen.
//
//   NEW (2025-10-23): End-of-match safety controls
//   • Start button toggles a latched STOP state (StopAll). When STOPPED, all powered
//     systems (drive, launcher, feed, intake) are forced off each loop and controls
//     are ignored. Press Start again to RESUME.
//   • Optional Auto-Stop timer (defaults disabled; 119s) starts at TeleOp INIT,
//     renders a top-line countdown (only when enabled), and calls StopAll at 0.
//
// CONTROLS (Gamepad 1):
//   Left stick ............. Forward/back + strafe
//   Right stick X .......... Rotation (disabled while AutoAim is ON or in grace)
//   Left trigger ........... Brake (reduces top speed toward slowestSpeed)
//   Right trigger .......... Manual launch RPM (only when AutoSpeed == false, not locked, not test)
//   LB ..................... Feed one ball — with Intake Assist if Intake is OFF
//   RB ..................... Toggle intake ON/OFF
//   Right Stick Btn ........ TOGGLE AutoAim  [double-pulse on ENABLE, single on DISABLE]
//                            (ENABLE only when goal tag is visible; 4s grace on loss)
//   Triangle / Y ........... TOGGLE AutoSpeed [double-pulse on ENABLE, single on DISABLE]
//   Square / X ............. TOGGLE Manual RPM LOCK (only when AutoSpeed == false)
//   Circle / B ............. EJECT (temporary RPM = EjectRPM; feeds once with Intake Assist; restores prior RPM)
//   D-pad Up ............... Enable RPM TEST MODE
//   D-pad Left/Right ....... -/+ 50 RPM while TEST MODE enabled (applies immediately)
//   D-pad Down ............. Disable RPM TEST MODE and STOP launcher
//   Start (G1 or G2) ....... TOGGLE StopAll latch (STOP ↔ RESUME)
//
// TELEMETRY:
//   Alliance, BrakeCap, Intake state, AutoSpeed, ManualLock, RT, RPM Target/Actual,
//   AutoAim Enabled, Tag Visible, Tag Heading (deg), Tag Distance (in/raw+smoothed),
//   AutoRPM In/Out, AutoRPM tunables/smoothing, AutoAim grace countdown.
//   NEW: When autoStopTimerEnabled == true, a top-line ⏱ AutoStop countdown is shown.
//
// NOTES:
//   - AutoAim enable requires a visible goal tag. If tag is lost, a GRACE TIMER runs
//     for autoAimLossGraceMs; tag seen again within grace = continue; else disable + pulse.
//   - While AutoAim is ON (including grace), right-stick rotation is ignored.
//   - Manual aim-window rumble runs ONLY when AutoAim is OFF.
//   - Rumble helpers use gamepad.rumble(left,right,durationMs) for SDK compatibility.
//   - StopAll latches powered systems OFF until Start is pressed again. A StopAll is
//     also invoked on OpMode.stop() for safety.
//
// AUTHOR:         Indianola Robotics – 2025 Season (DECODE)
// LAST UPDATED:   2025-10-23
// ============================================================================

// (header unchanged)
package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.drive.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Feed;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.vision.VisionAprilTag;
import org.firstinspires.ftc.teamcode.vision.TagAimController;
import org.firstinspires.ftc.teamcode.utils.ObeliskSignal;

import static java.lang.Math.*;

import org.firstinspires.ftc.teamcode.input.ControllerBindings;
import static org.firstinspires.ftc.teamcode.input.ControllerBindings.Pad;
import static org.firstinspires.ftc.teamcode.input.ControllerBindings.Btn;
import static org.firstinspires.ftc.teamcode.input.ControllerBindings.Trigger;

import org.firstinspires.ftc.teamcode.util.RumbleNotifier;

// AutoSpeed controller
import org.firstinspires.ftc.teamcode.control.LauncherAutoSpeedController;

// NEW: shared configs
import org.firstinspires.ftc.teamcode.config.AutoRpmConfig;
import org.firstinspires.ftc.teamcode.config.SharedRobotTuning;

import java.util.Locale;

public abstract class TeleOpAllianceBase extends OpMode {
    protected abstract Alliance alliance();

    // ---------------- Startup Defaults (edit here) ----------------
    private static final boolean DEFAULT_AUTOSPEED_ENABLED = false; // AutoSpeed OFF at start
    private static final boolean DEFAULT_AUTOAIM_ENABLED   = false; // AutoAim OFF at start
    private static final boolean DEFAULT_INTAKE_ENABLED    = false; // Intake OFF at start

    // ---------------- Subsystems ----------------
    protected Drivebase drive;
    protected Launcher launcher;
    protected Feed feed;
    protected Intake intake;

    // ---------------- Drivetrain Tunables ----------------
    private double slowestSpeed = 0.25;

    // ---------------- Launcher Manual Range (used when AutoSpeed == false) ----------------
    private double rpmBottom    = 0;      // If > 0, run at least this RPM in manual even with RT=0
    private double rpmTop       = 6000;

    // ---------------- State ----------------
    private boolean autoSpeedEnabled = DEFAULT_AUTOSPEED_ENABLED;
    private boolean autoAimEnabled   = DEFAULT_AUTOAIM_ENABLED;

    // Manual RPM Lock (Square/X) — only when AutoSpeed == false
    private boolean manualRpmLocked = false;
    private double  manualLockedRpm = 0.0;

    // ---------------- Vision + Aim ----------------
    private VisionAprilTag vision;
    private TagAimController aim = new TagAimController();

    // ---------------- AutoAim Loss Grace (CONFIGURABLE) ----------------
    private int  autoAimLossGraceMs = 4000; // 4s grace to reacquire tag before disabling
    private long aimLossStartMs = -1L;      // <0 means not in loss/grace

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
    private boolean aimRumbleEnabled       = true;
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

    // (REMOVED) TeleOp-local copies of Auto RPM curve & smoothing:
    //   autoNearDistIn, autoNearRpm, autoFarDistIn, autoFarRpm, autoSmoothingAlpha
    // These are now centralized in AutoRpmConfig.java and applied to autoCtrl.

    // ---------------- AutoSpeed seeding behavior ----------------
    // NOTE: This used to live here as InitialAutoDefaultSpeed — now central.
    private double InitialAutoDefaultSpeed = SharedRobotTuning.INITIAL_AUTO_DEFAULT_SPEED;
    private boolean autoHadTagFix = false;

    // ---------------- Intake Assist + Eject ----------------
    // NOTE: intakeAssistMs lived here before; now use SharedRobotTuning.INTAKE_ASSIST_MS
    private int    intakeAssistMs = SharedRobotTuning.INTAKE_ASSIST_MS;
    private double ejectRpm       = 300.0;
    private int    ejectTimeMs    = 300;

    // ---------------- NEW: StopAll / Latch & Auto-Stop Timer ----------------
    private boolean stopLatched = false;
    protected boolean autoStopTimerEnabled = false;
    protected int autoStopTimerTimeSec = 119;
    private long teleopInitMillis = 0L;
    private boolean autoStopTriggered = false;
    private boolean lastStartG1 = false, lastStartG2 = false;

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
        vision.setRangeScale(0.03); // calibration scale (keep or retune as needed)

        // ---- Controller Bindings Setup ----
        controls = new ControllerBindings();

        // ====== CONTROLLER BINDINGS (unchanged) ======
        controls.bindPress(Pad.G1, Btn.LB, () -> feedOnceWithIntakeAssist());
        controls.bindPress(Pad.G1, Btn.RB, () -> intake.toggle());

        controls.bindPress(Pad.G1, Btn.R_STICK_BTN, () -> {
            int targetId = (alliance() == Alliance.BLUE) ? VisionAprilTag.TAG_BLUE_GOAL : VisionAprilTag.TAG_RED_GOAL;
            AprilTagDetection detNow = vision.getDetectionFor(targetId);
            if (!autoAimEnabled) {
                if (detNow != null) { autoAimEnabled = true; aimLossStartMs = -1; pulseDouble(gamepad1); }
                else { pulseSingle(gamepad1); }
            } else { autoAimEnabled = false; aimLossStartMs = -1; pulseSingle(gamepad1); }
        });

        controls.bindPress(Pad.G1, Btn.Y, () -> {
            autoSpeedEnabled = !autoSpeedEnabled;
            ensureAutoCtrl();
            AutoRpmConfig.apply(autoCtrl);         // <— CENTRALIZED (was local fields)
            autoCtrl.setAutoEnabled(autoSpeedEnabled);

            if (autoSpeedEnabled) {
                autoHadTagFix = false;
                int targetId = (alliance() == Alliance.BLUE) ? VisionAprilTag.TAG_BLUE_GOAL : VisionAprilTag.TAG_RED_GOAL;
                AprilTagDetection detNow = vision.getDetectionFor(targetId);
                Double seedIn = getGoalDistanceInchesScaled(detNow);

                double seededRpm;
                if (seedIn != null) { seededRpm = autoCtrl.updateWithVision(seedIn); autoHadTagFix = true; }
                else { seededRpm = InitialAutoDefaultSpeed; }
                launcher.setTargetRpm(seededRpm);

                manualRpmLocked = false;
                pulseDouble(gamepad1);
            } else {
                autoCtrl.onManualOverride(launcher.getCurrentRpm());
                pulseSingle(gamepad1);
            }
        });

        controls.bindPress(Pad.G1, Btn.X, () -> {
            if (!autoSpeedEnabled && !rpmTestEnabled) {
                if (!manualRpmLocked) { manualRpmLocked = true; manualLockedRpm = launcher.targetRpm; pulseDouble(gamepad1); }
                else { manualRpmLocked = false; pulseSingle(gamepad1); }
            }
        });

        controls.bindPress(Pad.G1, Btn.B, () -> ejectOnce());

        controls.bindPress(Pad.G1, Btn.DPAD_UP,    () -> { rpmTestEnabled = true;  launcher.setTargetRpm(rpmTestTarget); });
        controls.bindPress(Pad.G1, Btn.DPAD_LEFT,  () -> { if (rpmTestEnabled) { rpmTestTarget = clamp(rpmTestTarget - 50.0, 0.0, 6000.0); launcher.setTargetRpm(rpmTestTarget); }});
        controls.bindPress(Pad.G1, Btn.DPAD_RIGHT, () -> { if (rpmTestEnabled) { rpmTestTarget = clamp(rpmTestTarget + 50.0, 0.0, 6000.0); launcher.setTargetRpm(rpmTestTarget); }});
        controls.bindPress(Pad.G1, Btn.DPAD_DOWN,  () -> { rpmTestEnabled = false; launcher.stop(); });

        controls.bindTriggerAxis(Pad.G1, Trigger.RT, (rt0to1) -> {
            if (!autoSpeedEnabled && !manualRpmLocked && !rpmTestEnabled) {
                double target = rpmBottom + rt0to1 * (rpmTop - rpmBottom);
                launcher.setTargetRpm(target);
            }
        });

        // Co-driver mirrors Y / LB / RB as before
        controls.bindPress(Pad.G2, Btn.LB, () -> feedOnceWithIntakeAssist());
        controls.bindPress(Pad.G2, Btn.RB, () -> intake.toggle());
        controls.bindPress(Pad.G2, Btn.Y,  () -> {
            autoSpeedEnabled = !autoSpeedEnabled;
            ensureAutoCtrl();
            AutoRpmConfig.apply(autoCtrl);         // <— CENTRALIZED
            autoCtrl.setAutoEnabled(autoSpeedEnabled);

            if (autoSpeedEnabled) {
                autoHadTagFix = false;
                int targetId = (alliance() == Alliance.BLUE) ? VisionAprilTag.TAG_BLUE_GOAL : VisionAprilTag.TAG_RED_GOAL;
                AprilTagDetection detNow = vision.getDetectionFor(targetId);
                Double seedIn = getGoalDistanceInchesScaled(detNow);

                double seededRpm;
                if (seedIn != null) { seededRpm = autoCtrl.updateWithVision(seedIn); autoHadTagFix = true; }
                else { seededRpm = InitialAutoDefaultSpeed; }
                launcher.setTargetRpm(seededRpm);

                manualRpmLocked = false;
                pulseDouble(gamepad1);
            } else {
                autoCtrl.onManualOverride(launcher.getCurrentRpm());
                pulseSingle(gamepad1);
            }
        });

        // Haptics / controller setup
        initAimRumble();

        // Auto RPM controller init (centralized apply)
        ensureAutoCtrl();
        AutoRpmConfig.apply(autoCtrl);             // <— CENTRALIZED
        autoCtrl.setAutoEnabled(autoSpeedEnabled);

        // TeleOp INIT timestamp for optional auto-stop
        teleopInitMillis = System.currentTimeMillis();
        autoStopTriggered = false;
        stopLatched = false;

        // FIRST-LINE telemetry
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
        // (unchanged input handling; omitted here for brevity — keep your original)

        // ... keep your existing loop body EXACTLY as before up to telemetry ...

        // === BEGIN of unchanged driving + AutoSpeed block ===
        // (Your original code here—no logic change; only differences are below in telemetry.)
        // === END of unchanged driving + AutoSpeed block ===

        if (vision != null) vision.observeObelisk();
        telemetry.addData("Obelisk", ObeliskSignal.getDisplay());

        telemetry.addData("Alliance", "%s", alliance());
        double brake = gamepad1.left_trigger;
        double cap = 1.0 - brake * (1.0 - slowestSpeed);
        telemetry.addData("BrakeCap", "%.2f", cap);
        telemetry.addData("Intake", intake.isOn() ? "ON" : "OFF");
        telemetry.addData("AutoSpeed", autoSpeedEnabled ? "ON" : "OFF");
        if (manualRpmLocked) telemetry.addData("ManualLock", "LOCKED (%.0f rpm)", manualLockedRpm);
        telemetry.addData("RT", "%.2f", gamepad1.right_trigger);
        telemetry.addData("RPM Target/Actual", "%.0f / %.0f", launcher.targetRpm, launcher.getCurrentRpm());

        // (Your existing AutoAim/Tag telemetry lines remain)

        // ---- AutoRPM tunables now reported from controller / config (CENTRALIZED) ----
        if (autoCtrl != null) {
            telemetry.addData("AutoRPM Tunables",
                    "Near=%.0f→%.0f  Far=%.0f→%.0f",
                    autoCtrl.getNearDistanceIn(), autoCtrl.getNearSpeedRpm(),
                    autoCtrl.getFarDistanceIn(),  autoCtrl.getFarSpeedRpm());
            telemetry.addData("AutoRPM Smoothing α", "%.2f", autoCtrl.getSmoothingAlpha());
        } else {
            telemetry.addData("AutoRPM Tunables (cfg)",
                    "Near=%.0f→%.0f  Far=%.0f→%.0f",
                    AutoRpmConfig.NEAR_DIST_IN, AutoRpmConfig.NEAR_RPM,
                    AutoRpmConfig.FAR_DIST_IN,  AutoRpmConfig.FAR_RPM);
            telemetry.addData("AutoRPM Smoothing α (cfg)", "%.2f", AutoRpmConfig.SMOOTH_ALPHA);
        }

        telemetry.update();
    }

    @Override
    public void stop() { stopAll(); }

    // ===== Helpers (unchanged, except intakeAssistMs comes from SharedRobotTuning) =====
    private void ensureAutoCtrl() { if (autoCtrl == null) autoCtrl = new LauncherAutoSpeedController(); }

    private void initAimRumble() {
        aimRumbleDriver1 = new RumbleNotifier(gamepad1);
        aimRumbleDriver1.setActive(true);
        aimRumbleDriver1.setThresholdDeg(aimRumbleDeg);
        aimRumbleDriver1.setMinMax(aimRumbleMinStrength, aimRumbleMaxStrength,
                aimRumbleMinPulseMs,  aimRumbleMaxPulseMs,
                aimRumbleMinCooldownMs, aimRumbleMaxCooldownMs);
    }

    private void pulseDouble(Gamepad gp) { gp.rumble((float)0.8, (float)0.8, 120); sleepMs(80); gp.rumble((float)0.8, (float)0.8, 120); }
    private void pulseSingle(Gamepad gp) { gp.rumble((float)0.8, (float)0.8, 150); }

    private Double getGoalDistanceInchesScaled(AprilTagDetection det) {
        if (det == null) return null;
        double rM_sc = vision.getScaledRange(det);
        if (Double.isNaN(rM_sc) || !Double.isFinite(rM_sc)) return null;
        return rM_sc * M_TO_IN;
    }

    private void feedOnceWithIntakeAssist() {
        boolean wasOn = intake.isOn();
        if (!wasOn) intake.set(true);
        feed.feedOnceBlocking();
        if (!wasOn) {
            sleepMs(intakeAssistMs); // now set from SharedRobotTuning
            intake.set(false);
        }
    }

    private void ejectOnce() {
        double prevCmd = launcher.targetRpm;
        double tempCmd = clamp(ejectRpm, 0, rpmTop);
        launcher.setTargetRpm(tempCmd);
        sleepMs(Math.max(100, ejectTimeMs / 3));
        boolean wasOn = intake.isOn();
        if (!wasOn) intake.set(true);
        feed.feedOnceBlocking();
        sleepMs(ejectTimeMs);
        if (!wasOn) { sleepMs(intakeAssistMs); intake.set(false); }
        launcher.setTargetRpm(prevCmd);
    }

    private void sleepMs(int ms) { try { Thread.sleep(ms); } catch (InterruptedException ignored) {} }

    protected void stopAll() {
        try { drive.stop(); } catch (Throwable t) { try { drive.drive(0,0,0); } catch (Throwable ignored) {} }
        try { launcher.stop(); } catch (Throwable t) { try { launcher.setTargetRpm(0); } catch (Throwable ignored) {} }
        try { feed.stop(); } catch (Throwable t) { try { feed.setPower(0); } catch (Throwable ignored) {} }
        try { intake.stop(); } catch (Throwable t) { try { intake.set(false); } catch (Throwable ignored) {} }
    }

    private void onStoppedLoopHold() { stopAll(); telemetry.addLine("⛔ STOPPED — press START to RESUME"); telemetry.update(); }
}
