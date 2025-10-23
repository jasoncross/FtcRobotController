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
//
// TELEMETRY:
//   Alliance, BrakeCap, Intake state, AutoSpeed, ManualLock, RT, RPM Target/Actual,
//   AutoAim Enabled, Tag Visible, Tag Heading (deg), Tag Distance (in/raw+smoothed),
//   AutoRPM In/Out, AutoRPM tunables/smoothing, AutoAim grace countdown.
//
// NOTES:
//   - AutoAim enable requires a visible goal tag. If tag is lost, a GRACE TIMER runs
//     for autoAimLossGraceMs; tag seen again within grace = continue; else disable + pulse.
//   - While AutoAim is ON (including grace), right-stick rotation is ignored.
//   - Manual aim-window rumble runs ONLY when AutoAim is OFF.
//   - Rumble helpers use gamepad.rumble(left,right,durationMs) for SDK compatibility.
//
// AUTHOR:         Indianola Robotics – 2025 Season (DECODE)
// LAST UPDATED:   2025-10-23
// ============================================================================
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
import org.firstinspires.ftc.teamcode.util.ObeliskSignal;

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

    // INITIAL TUNABLES for Auto RPM
    private double autoNearDistIn = 24.0;
    private double autoNearRpm    = 1000.0;
    private double autoFarDistIn  = 120.0;
    private double autoFarRpm     = 4500.0;
    private double autoSmoothingAlpha = 0.15;

    // ---------------- AutoSpeed seeding behavior ----------------
    private double InitialAutoDefaultSpeed = 2500.0;  // RPM used ONLY before first tag lock after enabling AutoSpeed
    private boolean autoHadTagFix = false;            // becomes true after AutoSpeed sees a tag at least once

    // ---------------- Intake Assist + Eject ----------------
    private int    intakeAssistMs = 250;   // ms to run intake if it was OFF during a feed/eject
    private double ejectRpm       = 300.0; // temporary launcher RPM used during eject
    private int    ejectTimeMs    = 300;   // how long to hold eject RPM while feeding

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
        vision.setRangeScale(0.03); // keep your calibration scale unless re-tuned

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
        controls.bindPress(Pad.G1, Btn.DPAD_LEFT,  () -> { if (rpmTestEnabled) { rpmTestTarget = clamp(rpmTestTarget - RPM_TEST_STEP, RPM_TEST_MIN, RPM_TEST_MAX); launcher.setTargetRpm(rpmTestTarget); }});
        controls.bindPress(Pad.G1, Btn.DPAD_RIGHT, () -> { if (rpmTestEnabled) { rpmTestTarget = clamp(rpmTestTarget + RPM_TEST_STEP, RPM_TEST_MIN, RPM_TEST_MAX); launcher.setTargetRpm(rpmTestTarget); }});
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
        autoCtrl.setAutoEnabled(autoSpeedEnabled);

        // ---- FIRST LINE telemetry (init): obelisk memory ----
        telemetry.addData("Obelisk", ObeliskSignal.getDisplay());

        telemetry.addData("TeleOp", "Alliance: %s", alliance());
        telemetry.addData("Startup Defaults", "AutoSpeed=%s  AutoAim=%s  Intake=%s",
                DEFAULT_AUTOSPEED_ENABLED ? "ON" : "OFF",
                DEFAULT_AUTOAIM_ENABLED   ? "ON" : "OFF",
                DEFAULT_INTAKE_ENABLED    ? "ON" : "OFF");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update bindings
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

        // Alliance target
        int targetId = (alliance() == Alliance.BLUE)
                ? VisionAprilTag.TAG_BLUE_GOAL
                : VisionAprilTag.TAG_RED_GOAL;

        // Vision
        AprilTagDetection goalDet = vision.getDetectionFor(targetId);

        if (goalDet != null) {
            double hDeg  = goalDet.ftcPose.bearing;
            double rM_sc = vision.getScaledRange(goalDet);
            smHeadingDeg = (smHeadingDeg == null) ? hDeg : (SMOOTH_A * hDeg + (1 - SMOOTH_A) * smHeadingDeg);
            if (!Double.isNaN(rM_sc) && Double.isFinite(rM_sc)) {
                smRangeMeters = (smRangeMeters == null) ? rM_sc : (SMOOTH_A * rM_sc + (1 - SMOOTH_A) * smRangeMeters);
            }
        }

        // AutoAim + grace handling
        long now = System.currentTimeMillis();
        if (autoAimEnabled) {
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
            autoCtrl.setParams(autoNearDistIn, autoNearRpm, autoFarDistIn, autoFarRpm);
            autoCtrl.setSmoothingAlpha(autoSmoothingAlpha);

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
                    autoNearDistIn, autoNearRpm, autoFarDistIn, autoFarRpm);
            telemetry.addData("AutoRPM Smoothing α", "%.2f", autoSmoothingAlpha);
        }
        telemetry.update();
    }

    // =========================================================================
    // HELPERS
    // =========================================================================
    private void ensureAutoCtrl() {
        if (autoCtrl == null) autoCtrl = new LauncherAutoSpeedController();
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
}
