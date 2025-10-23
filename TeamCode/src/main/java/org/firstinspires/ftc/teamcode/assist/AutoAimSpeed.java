// ============================================================================
// FILE:           AutoAimSpeed.java
// LOCATION:       TeamCode/src/main/java/org/firstinspires/ftc/teamcode/assist/
//
// PURPOSE:
//   Shared helper for BOTH TeleOp and Autonomous:
//   - Uses TagAimController to compute aim (twist) toward the goal tag
//   - Uses LauncherAutoSpeedController to compute launcher RPM from distance
//   - Holds last known distance if tag is lost; seeds with InitialAutoDefaultSpeed
//
// HISTORY NOTES (TUNING LOCATIONS):
//   • initialAutoDefaultSpeed previously tuned inside TeleOpAllianceBase.java
//     (InitialAutoDefaultSpeed) — now centralized in SharedRobotTuning.
//   • maxTwist / rpmTolerance used to be local to BaseAuto — now centralized.
//
// ============================================================================
package org.firstinspires.ftc.teamcode.assist;

import org.firstinspires.ftc.teamcode.config.SharedRobotTuning;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.vision.TagAimController;
import org.firstinspires.ftc.teamcode.vision.VisionAprilTag;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.control.LauncherAutoSpeedController;

public class AutoAimSpeed {

    // ---------------- Tunables (centralized) ----------------
    public double maxTwist = SharedRobotTuning.TURN_TWIST_CAP;             // cap for twist magnitude
    public double rpmTolerance = SharedRobotTuning.RPM_TOLERANCE;          // ± tolerance for "at speed"
    public double initialAutoDefaultSpeed = SharedRobotTuning.INITIAL_AUTO_DEFAULT_SPEED;

    private static final double M_TO_IN = 39.37007874015748;

    // ---------------- Deps ----------------
    private final VisionAprilTag vision;
    private final TagAimController aim;
    private final LauncherAutoSpeedController autoCtrl;
    private final Launcher launcher;

    // ---------------- State ----------------
    private boolean enabled = false;
    private boolean hadTagFix = false;

    public AutoAimSpeed(VisionAprilTag vision,
                        TagAimController aim,
                        LauncherAutoSpeedController autoCtrl,
                        Launcher launcher) {
        this.vision   = vision;
        this.aim      = aim;
        this.autoCtrl = autoCtrl;
        this.launcher = launcher;
    }

    public void enable()  {
        enabled = true;
        try { autoCtrl.setAutoEnabled(true); } catch (Throwable ignored) {}
    }

    public void disable() {
        enabled = false;
        try { autoCtrl.onManualOverride(launcher.getCurrentRpm()); } catch (Throwable ignored) {}
        try { autoCtrl.setAutoEnabled(false); } catch (Throwable ignored) {}
    }

    public boolean isEnabled() { return enabled; }

    /** Update loop: set RPM from distance and return suggested twist. */
    public double update(AprilTagDetection det, double currentHeadingDeg) {
        if (!enabled) return 0;

        // ---- AutoSpeed ----
        double outRpm;
        if (det != null) {
            double rM = vision.getScaledRange(det);
            if (!Double.isNaN(rM) && Double.isFinite(rM)) {
                double distIn = rM * M_TO_IN;
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
        if (det != null) twist = aim.turnPower(det);

        // Clamp twist
        if (twist >  maxTwist) twist =  maxTwist;
        if (twist < -maxTwist) twist = -maxTwist;
        return twist;
    }

    /** True when launcher RPM is within ±tolRpm (or default rpmTolerance). */
    public boolean atSpeed(Double tolRpm) {
        double tol = (tolRpm != null && tolRpm > 0) ? tolRpm : rpmTolerance;
        return launcher.atSpeed(tol);
    }
}
