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
import org.firstinspires.ftc.teamcode.config.DriveTuning;
import org.firstinspires.ftc.teamcode.config.SharedRobotTuning;

import static java.lang.Math.*;

/*
 * FILE: Drivebase.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/
 *
 * PURPOSE
 *   - Provide the shared mecanum drivetrain abstraction for BOTH TeleOp and
 *     Autonomous, including encoder-based translations and IMU-controlled turns.
 *   - Normalize driver stick inputs, apply strafing compensation, and expose the
 *     heading needed by BaseAuto and AutoAim helpers.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Drivetrain motion & positioning)
 *   - WHEEL_DIAMETER_IN / TICKS_PER_REV / GEAR_RATIO
 *       • Define the physical conversion between encoder ticks and inches.
 *       • Update whenever wheels or cartridges change so move() remains accurate.
 *   - STRAFE_CORRECTION
 *       • Empirical multiplier (typically 1.05–1.25) compensating for mecanum
 *         under-travel during lateral moves. Impacts both TeleOp strafing and
 *         BaseAuto move() helpers.
 *   - TURN_KP / TURN_KD
 *       • PD gains for IMU-based turn(). Coordinate with SharedRobotTuning
 *         LOCK_TOLERANCE_DEG and TURN_TWIST_CAP so Auto unwinds smoothly.
 *   - TURN_TOLERANCE_DEG / TURN_SETTLE_TIME
 *       • Define how tightly turn() locks onto the target heading and how long it
 *         must remain there before declaring success. Keep aligned with
 *         SharedRobotTuning.LOCK_TOLERANCE_DEG when autos depend on precise aim.
 *
 * METHODS
 *   - drive(drive, strafe, twist)
 *       • Robot-centric TeleOp drive with power normalization.
 *   - move(distance, headingDeg, power)
 *       • Encoder translation along a field direction (0°=forward, +90°=right).
 *   - turn(degrees, power)
 *       • IMU-driven rotation using the PD gains + settle timers above.
 *   - heading()
 *       • Returns IMU yaw in degrees (CCW positive) for use in field-centric math.
 *   - stop()/stopAll()
 *       • Halt all motors—BaseAuto safety routines call these frequently.
 *
 * NOTES
 *   - IMU orientation defaults to LOGO UP, USB RIGHT (FTC standard). Adjust
 *     `config/SharedRobotTuning` if the control hub is remounted so heading()
 *     increases CCW as expected by TeleOp + Auto helpers.
 *   - Motor names must match Robot Controller configuration exactly:
 *     "FrontLeft", "FrontRight", "BackLeft", "BackRight".
 *   - After wiring or configuration changes, push the robot forward by hand—all
 *     encoders should increase. If not, flip the affected motor direction.
 *   - Driver stick mapping: drive = +leftY, strafe = -leftX, twist = -rightX to
 *     preserve historical control feel described in DECODE_Season_Context.md.
 */

public class Drivebase {

    // ======= TUNE THESE =======
    public static final double WHEEL_DIAMETER_IN = DriveTuning.WHEEL_DIAMETER_IN; // goBILDA 96mm wheel ≈ 3.7795"
    public static final double TICKS_PER_REV     = DriveTuning.TICKS_PER_REV;     // goBILDA 5202 312RPM output encoder
    public static final double GEAR_RATIO        = DriveTuning.GEAR_RATIO;        // wheel revs per motor rev (set >1 if reduced)
    public static final double TICKS_PER_IN      = (TICKS_PER_REV * GEAR_RATIO) / (Math.PI * WHEEL_DIAMETER_IN);

    // Strafing compensation for lateral under-travel (tune on your field surface)
    public static final double STRAFE_CORRECTION = DriveTuning.STRAFE_CORRECTION;

    // IMU turn control gains + tolerance
    public static final double TURN_KP = DriveTuning.TURN_KP;
    public static final double TURN_KD = DriveTuning.TURN_KD;
    public static final double TURN_TOLERANCE_DEG = DriveTuning.TURN_TOLERANCE_DEG;   // stop when within ±tuned tolerance
    public static final double TURN_SETTLE_TIME   = DriveTuning.TURN_SETTLE_TIME_SEC;  // remain within tolerance this many seconds

    // ======= INTERNAL =======
    private final LinearOpMode linear;   // Non-null only in Autonomous usage
    private final Telemetry telemetry;

    private final DcMotorEx fl, fr, bl, br;
    private final IMU imu;

    // Remember baseline run mode to restore after RUN_TO_POSITION moves (Auto)
    private DcMotor.RunMode baseRunModeAfterMove = DcMotor.RunMode.RUN_USING_ENCODER;

    // ---------- Constructor for AUTONOMOUS (blocking helpers allowed) ----------
    // CHANGES (2025-10-31): Added safeInit to guarantee zero drive power during INIT.

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
        safeInit();
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
        safeInit();
    }

    /** Ensure all drive motors are zeroed during INIT (no unintended motion). */
    public void safeInit() {
        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) {
            m.setPower(0.0);
        }
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

        // ---- IMU orientation (tunable via SharedRobotTuning) ----
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        SharedRobotTuning.LOGO_DIRECTION,
                        SharedRobotTuning.USB_DIRECTION
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
        double y =  cos(rad); // +forward   (0° = forward)
        double x =  sin(rad); // +right     (+90° = right)


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

    /** Stop all drive motors. (Alias retained for compatibility with TeleOp StopAll.) */
    public void stop() { stopAll(); }

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
