/*
 * FILE: AutoAimSpeed.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/assist/
 *
 * PURPOSE
 *   - Share one AprilTag-powered "Auto Aim + Auto Speed" loop between
 *     TeleOpAllianceBase and every BaseAuto-derived routine so students tune a
 *     single behavior for both match phases.
 *   - Translate vision-provided distance into Launcher RPM through
 *     LauncherAutoSpeedController while guarding against noisy detections.
 *   - Smoothly clamp twist corrections (fed into Drivebase) so the driver keeps
 *     control while automation nudges the robot toward the goal.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → AutoAim section)
 *   - maxTwist (defaults to AutoAimTuning.MAX_TWIST)
 *       • Caps the steering command returned from update().
 *       • AutoAimTuning mirrors the shared twist cap by default; override this
 *         field when AutoAim should react faster than BaseAuto turns.
 *   - rpmTolerance (defaults to AutoAimTuning.RPM_TOLERANCE)
 *       • Locks in the readiness window used by atSpeed().
 *       • Coordinate with Launcher.atSpeedToleranceRPM so AutoAim and manual
 *         fire buttons agree on "ready." SharedRobotTuning overrides
 *         BaseAuto.rpmTol() as well.
 *   - initialAutoDefaultSpeed (defaults to AutoAimTuning.INITIAL_AUTO_DEFAULT_SPEED)
 *       • Seeds the launcher RPM before any tag is seen.
 *       • TeleOpAllianceBase may override its own copy for driver preference;
 *         mirror that value here if TeleOp diverges from the shared default.
 *
 * METHODS
 *   - AutoAimSpeed(...)
 *       • Constructor wiring vision, TagAimController, AutoSpeed curve helper,
 *         and the Launcher subsystem.
 *   - enable() / disable()
 *       • Toggle the assist loop and synchronize LauncherAutoSpeedController
 *         state with manual overrides.
 *   - isEnabled()
 *       • Convenience getter for OpModes managing toggle buttons.
 *   - update(AprilTagDetection, headingDeg)
 *       • Convert the latest provider-sourced target info into RPM + twist
 *         suggestions, falling back to the seeded RPM when tags are lost.
 *   - atSpeed(tolRpm)
 *       • Confirms launcher readiness using either the caller-provided tolerance
 *         or the shared rpmTolerance field.
 *
 * NOTES
 *   - AutoRpmConfig.apply(...) overwrites LauncherAutoSpeedController tunables
 *     during robot init; this class simply reads the live controller values.
 *   - Provider-supplied distance calibration changes feed directly into
 *     update(); re-run tape-measure checks after moving the camera or goal.
 *   - TagAimController limits are slightly wider than maxTwist so this class is
 *     the final clamp before we pass steering hints back to TeleOp/Auto.
 * CHANGES (2025-12-11): Routed AutoAimSpeed distance/heading queries through
 *                       VisionTargetProvider so the helper follows whichever
 *                       vision source is active (Limelight by default) while
 *                       preserving existing toggles and readiness behavior.
 */
package org.firstinspires.ftc.teamcode.assist;

import org.firstinspires.ftc.teamcode.config.AutoAimTuning;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.vision.TagAimController;
import org.firstinspires.ftc.teamcode.vision.VisionTargetProvider;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.control.LauncherAutoSpeedController;

public class AutoAimSpeed {

    // ---------------- Tunables (copied from SharedRobotTuning unless overridden) ----------------
    public double maxTwist = AutoAimTuning.MAX_TWIST;             // Clamp for twist suggestions; AutoAimTuning defaults to shared value
    public double rpmTolerance = AutoAimTuning.RPM_TOLERANCE;     // Shared readiness window so Auto and TeleOp agree on launcher "ready"
    public double initialAutoDefaultSpeed = AutoAimTuning.INITIAL_AUTO_DEFAULT_SPEED; // Seed RPM prior to tag lock; match TeleOpAllianceBase override if changed

    private static final double M_TO_IN = 39.37007874015748;               // Constant to convert meters → inches

    // ---------------- Deps ----------------
    private VisionTargetProvider provider;               // Shared target provider (distance + bearing)
    private final TagAimController aim;                  // PD twist controller for goal alignment
    private final LauncherAutoSpeedController autoCtrl;  // Distance→RPM curve helper
    private final Launcher launcher;                     // Physical flywheel subsystem

    // ---------------- State ----------------
    private boolean enabled = false;                     // Tracks whether assists are actively running
    private boolean hadTagFix = false;                   // Remembers if we have seen a tag this enable cycle

    // Constructor wires dependencies so both TeleOp and Auto share identical assists.
    public AutoAimSpeed(VisionTargetProvider provider,
                        TagAimController aim,
                        LauncherAutoSpeedController autoCtrl,
                        Launcher launcher) {
        this.provider = provider;
        this.aim      = aim;
        this.autoCtrl = autoCtrl;
        this.launcher = launcher;
        if (this.aim != null) {
            this.aim.setProvider(provider);
        }
    }

    /** Legacy-compatible setter so existing OpModes can swap providers without changing constructor signatures. */
    public void setProvider(VisionTargetProvider provider) {
        this.provider = provider;
        if (aim != null) {
            aim.setProvider(provider);
        }
    }

    // Enable the assist loop and let AutoSpeed control RPM again.
    public void enable()  {
        enabled = true;
        try { autoCtrl.setAutoEnabled(true); } catch (Throwable ignored) {}
    }

    // Disable assist, handing control back to manual RPM while preserving spin.
    public void disable() {
        enabled = false;
        try { autoCtrl.onManualOverride(launcher.getCurrentRpm()); } catch (Throwable ignored) {}
        try { autoCtrl.setAutoEnabled(false); } catch (Throwable ignored) {}
    }

    // Quick getter so OpModes can guard update() without extra state.
    public boolean isEnabled() { return enabled; }

    /** Update loop: set RPM from distance and return suggested twist. */
    public double update(AprilTagDetection det, double currentHeadingDeg) {
        if (!enabled) return 0;

        VisionTargetProvider vision = provider;
        if (vision == null && det != null) {
            vision = wrapDetection(det);
        }

        boolean hasTarget = vision != null && vision.hasGoalTarget();

        // ---- AutoSpeed ----
        double outRpm;
        if (hasTarget) {
            double distM = vision.getDistanceMeters();
            if (!Double.isNaN(distM) && Double.isFinite(distM)) {
                double distIn = distM * M_TO_IN;
                outRpm = autoCtrl.updateWithVision(distIn);
                hadTagFix = true;
            } else {
                outRpm = hadTagFix ? autoCtrl.updateWithVision(null) : initialAutoDefaultSpeed;
            }
        } else {
            outRpm = hadTagFix ? autoCtrl.updateWithVision(null) : initialAutoDefaultSpeed;
        }
        launcher.setTargetRpm(outRpm);

        // ---- AutoAim → twist suggestion ----
        double twist = 0.0;
        if (aim != null) {
            twist = provider != null ? aim.turnPower() : aim.turnPower(det);
        }

        // Clamp twist
        if (twist >  maxTwist) twist =  maxTwist;
        if (twist < -maxTwist) twist = -maxTwist;
        return twist;
    }

    /** True when launcher RPM is within ±tolRpm (or default rpmTolerance). */
    public boolean atSpeed(Double tolRpm) {
        double tol = (tolRpm != null && tolRpm > 0) ? tolRpm : rpmTolerance;
        double err = Math.abs(launcher.getCurrentRpm() - launcher.targetRpm);
        return err <= tol;
    }

    private VisionTargetProvider wrapDetection(AprilTagDetection det) {
        if (det == null) {
            return null;
        }
        return new VisionTargetProvider() {
            @Override
            public boolean hasTarget() {
                return true;
            }

            @Override
            public boolean hasGoalTarget() { return hasTarget(); }

            @Override
            public boolean hasAnyTarget() { return hasTarget(); }

            @Override
            public int getBestVisibleTagId() { return det.id; }

            @Override
            public double getHeadingErrorDeg() {
                return det.ftcPose.bearing;
            }

            @Override
            public double getDistanceMeters() {
                return det.ftcPose.range;
            }
        };
    }
}
