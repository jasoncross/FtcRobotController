package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import static java.lang.Math.*;

/*
 * FILE: Drivebase.java
 * LOCATION: teamcode/.../drive/
 *
 * PURPOSE:
 * - Shared mecanum drive for TeleOp + Autonomous with encoder support.
 * - TeleOp baseline mode: RUN_USING_ENCODER (hub velocity loop enabled).
 * - Autonomous: encoder translation via RUN_TO_POSITION and IMU-based turning.
 *
 * IMU ORIENTATION (VERY IMPORTANT):
 * - Your hub is mounted: LOGO/Label = UP, USB = RIGHT.
 *   This is configured below so heading() is correct (CCW positive).
 *
 * MOTOR NAMES (Robot Controller configuration must match EXACTLY):
 * - "FrontLeft", "FrontRight", "BackLeft", "BackRight"
 *
 * ENCODER PHASE ASSUMPTION:
 * - Pushing the robot FORWARD by hand should make all four encoder positions INCREASE.
 *   (You fixed FR/BR cabling; if this ever changes, flip that motor's setDirection().)
 *
 * TUNABLES (WHAT & WHY):
 * - WHEEL_DIAMETER_IN: Real wheel diameter (inches). Affects distance accuracy of move().
 * - TICKS_PER_REV: Encoder ticks per wheel/output shaft revolution (e.g., 537.7 for 5202 312RPM).
 * - GEAR_RATIO: Use >1.0 if external reduction makes the wheel turn slower than the motor.
 * - STRAFE_CORRECTION: Strafing under-travels on mecanum; 1.10–1.25 typical on carpet.
 * - TURN_KP, TURN_KD: IMU PD gains. Raise KP for faster correction; add KD to reduce overshoot.
 * - TURN_TOLERANCE_DEG, TURN_SETTLE_TIME: Stop criteria for turn() (accuracy & stability).
 *
 * IMPORTANT METHODS:
 * - drive(drive, strafe, twist): Robot-centric TeleOp drive with normalization.
 * - move(distanceInches, degrees, speed): Encoder translation (no rotation). 0°=forward, +90°=right.
 * - turn(degrees, speed): IMU PD rotation (+CCW), with tolerance & settle time.
 * - heading(): Yaw in degrees (CCW positive).
 *
 * STICK SIGNS (recommended outside this class to match your original OpMode):
 * - drive  =  +left_stick_y
 * - strafe =  -left_stick_x   (invert so right on stick = +strafe)
 * - twist  =  -right_stick_x  (+CCW)
 */

public class Drivebase {

    // ======= TUNE THESE =======
    public static final double WHEEL_DIAMETER_IN = 3.7795; // goBILDA 96mm wheel ≈ 3.7795"
    public static final double TICKS_PER_REV     = 537.7;  // goBILDA 5202 312RPM output encoder
    public static final double GEAR_RATIO        = 1.0;    // wheel revs per motor rev (set >1 if reduced)
    public static final double TICKS_PER_IN      = (TICKS_PER_REV * GEAR_RATIO) / (Math.PI * WHEEL_DIAMETER_IN);

    // Strafing compensation for lateral under-travel (tune on your field surface)
    public static final double STRAFE_CORRECTION = 1.15;

    // IMU turn control gains + tolerance
    public static final double TURN_KP = 0.012;
    public static final double TURN_KD = 0.003;
    public static final double TURN_TOLERANCE_DEG = 1.0;   // stop when within ±1°
    public static final double TURN_SETTLE_TIME   = 0.15;  // remain within tolerance this many seconds

    // ======= INTERNAL =======
    private final LinearOpMode linear;   // Non-null only in Autonomous usage
    private final Telemetry telemetry;

    private final DcMotorEx fl, fr, bl, br;
    private final IMU imu;

    // Remember baseline run mode to restore after RUN_TO_POSITION moves (Auto)
    private DcMotor.RunMode baseRunModeAfterMove = DcMotor.RunMode.RUN_USING_ENCODER;

    // ---------- Constructor for AUTONOMOUS (blocking helpers allowed) ----------
    public Drivebase(LinearOpMode op) {
        this.linear = op;
        this.telemetry = op.telemetry;
        HardwareMap hw = op.hardwareMap;

        imu  = hw.get(IMU.class, "imu");
        fl   = hw.get(DcMotorEx.class, "FrontLeft");
        fr   = hw.get(DcMotorEx.class, "FrontRight");
        bl   = hw.get(DcMotorEx.class, "BackLeft");
        br   = hw.get(DcMotorEx.class, "BackRight");

        commonInit(/*teleOp=*/false);
    }

    // ---------- Constructor for TELEOP ----------
    public Drivebase(HardwareMap hw, Telemetry telemetry) {
        this.linear = null; // TeleOp
        this.telemetry = telemetry;

        imu  = hw.get(IMU.class, "imu");
        fl   = hw.get(DcMotorEx.class, "FrontLeft");
        fr   = hw.get(DcMotorEx.class, "FrontRight");
        bl   = hw.get(DcMotorEx.class, "BackLeft");
        br   = hw.get(DcMotorEx.class, "BackRight");

        commonInit(/*teleOp=*/true);
    }

    /**
     * Shared hardware init. TeleOp baseline uses RUN_USING_ENCODER.
     * IMU is initialized with physical mount: LOGO UP, USB RIGHT.
     */
    private void commonInit(boolean teleOp) {
        // ---- Motor directions (standard mecanum). Flip a single motor here if needed. ----
        // ASSUMPTION: with these directions, pushing robot FORWARD increases all encoder counts.
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        // ---- Zero power behavior + reset encoders once ----
        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        // TeleOp baseline = RUN_USING_ENCODER (velocity loop); Auto baseline also RUN_USING_ENCODER
        DcMotor.RunMode base = DcMotor.RunMode.RUN_USING_ENCODER;
        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) {
            m.setMode(base);
        }
        baseRunModeAfterMove = base;

        // ---- IMU orientation: LOGO UP, USB RIGHT ----
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        ));
        resetHeading();
    }

    // =====================================================================================
    // TeleOp robot-centric drive (call every loop)
    // =====================================================================================
    public void drive(double drive, double strafe, double twist) {
        // Standard mecanum power sums (do not add extra per-motor sign flips here)
        double flP = drive + strafe + twist;
        double frP = drive - strafe - twist;
        double blP = drive - strafe + twist;
        double brP = drive + strafe - twist;

        // Normalize so the largest magnitude is 1.0
        double maxMag = max(1.0, max(abs(flP), max(abs(frP), max(abs(blP), abs(brP)))));
        fl.setPower(flP / maxMag);
        fr.setPower(frP / maxMag);
        bl.setPower(blP / maxMag);
        br.setPower(brP / maxMag);
    }

    // =====================================================================================
    // Autonomous helpers (blocking). In TeleOp, these NO-OP to avoid freezes.
    // =====================================================================================

    /**
     * Encoder-based translation without rotation.
     *
     * @param distanceInches distance to move (inches)
     * @param degrees        heading of translation: 0=forward, +90=right, 180=back, -90=left
     * @param speed          power cap 0..1 (0.3–0.7 typical for accuracy)
     */
    public void move(double distanceInches, double degrees, double speed) {
        if (!isActive()) return;  // TeleOp: don't block

        speed = clamp(speed, 0.1, 1.0);

        // Polar (robot-centric) with 0°=forward
        double rad = toRadians(degrees);
        double x =  cos(rad); // +right
        double y =  sin(rad); // +forward

        // Compensate lateral losses
        double xAdj = x * STRAFE_CORRECTION;
        double yAdj = y;

        // Wheel multipliers (no twist)
        double flMult = yAdj + xAdj;
        double frMult = yAdj - xAdj;
        double blMult = yAdj - xAdj;
        double brMult = yAdj + xAdj;

        // Normalize multipliers
        double maxMag = max(1.0, max(abs(flMult), max(abs(frMult), max(abs(blMult), abs(brMult)))));
        flMult /= maxMag; frMult /= maxMag; blMult /= maxMag; brMult /= maxMag;

        int baseTicks = (int) round(distanceInches * TICKS_PER_IN);
        int flT = fl.getCurrentPosition() + (int) round(baseTicks * flMult);
        int frT = fr.getCurrentPosition() + (int) round(baseTicks * frMult);
        int blT = bl.getCurrentPosition() + (int) round(baseTicks * blMult);
        int brT = br.getCurrentPosition() + (int) round(baseTicks * brMult);

        setRunToPosition(flT, frT, blT, brT);
        setAllPower(speed);

        while (isActive() && (fl.isBusy() || fr.isBusy() || bl.isBusy() || br.isBusy())) {
            idle(); // yield to SDK
        }
        restoreBaseRunMode(); // back to RUN_USING_ENCODER
    }

    /**
     * IMU-based in-place rotation (relative).
     * @param degrees +CCW / -CW
     * @param speed   power cap 0..1 (0.3–0.6 typical)
     */
    public void turn(double degrees, double speed) {
        if (!isActive()) return;  // TeleOp: don't block

        speed = clamp(speed, 0.2, 1.0);
        double start = heading();
        double target = normDeg(start + degrees);

        ElapsedTime dt = new ElapsedTime();
        double lastErr = 0, settleStart = -1;

        while (isActive()) {
            double cur = heading();
            double err = shortestDiff(target, cur);
            double derr = (err - lastErr) / max(1e-3, dt.seconds());
            lastErr = err; dt.reset();

            double cmd = TURN_KP * err + TURN_KD * derr;
            cmd = clamp(cmd, -speed, speed);

            fl.setPower(cmd); bl.setPower(cmd);
            fr.setPower(-cmd); br.setPower(-cmd);

            if (abs(err) <= TURN_TOLERANCE_DEG) {
                if (settleStart < 0) settleStart = now();
                if ((now() - settleStart) >= TURN_SETTLE_TIME) break;
            } else {
                settleStart = -1;
            }
            idle();
        }
        stopAll();
    }

    // =====================================================================================
    // Utility / helpers
    // =====================================================================================

    /** Zero the robot's yaw so heading() returns ~0 from this moment forward. */
    public void resetHeading() { imu.resetYaw(); }

    /** Current yaw in DEGREES, CCW positive. */
    public double heading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    /** Immediately sets all four powers. */
    public void setAllPower(double p) {
        fl.setPower(p); fr.setPower(p); bl.setPower(p); br.setPower(p);
    }

    /** Stop all drive motors. */
    public void stopAll() { setAllPower(0); }

    // --- internal move/turn helpers ---
    private void setRunToPosition(int flT, int frT, int blT, int brT) {
        fl.setTargetPosition(flT); fr.setTargetPosition(frT);
        bl.setTargetPosition(blT); br.setTargetPosition(brT);
        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) {
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    private void restoreBaseRunMode() {
        stopAll();
        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) {
            m.setMode(baseRunModeAfterMove); // RUN_USING_ENCODER
        }
    }

    // --- op-mode context ---
    private boolean isActive() { return linear != null && linear.opModeIsActive(); }
    private void idle() { if (linear != null) linear.idle(); }
    private static double now() { return System.nanoTime() / 1e9; }

    // --- math helpers ---
    private static double shortestDiff(double target, double current) {
        double d = normDeg(target - current);
        if (d > 180) d -= 360;
        if (d < -180) d += 360;
        return d;
    }
    private static double normDeg(double a) {
        double r = a % 360;
        if (r < 0) r += 360;
        return r;
    }
    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
