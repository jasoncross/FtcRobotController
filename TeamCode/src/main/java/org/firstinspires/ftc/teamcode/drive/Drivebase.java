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
    public static final double TICKS_PER_REV     = DriveTuning.TICKS_PER_REV;     // Encoder ticks per motor revolution
    public static final double GEAR_RATIO        = DriveTuning.GEAR_RATIO;        // wheel revs per motor rev (set >1 if reduced)
    public static final double TICKS_PER_IN      = (TICKS_PER_REV * GEAR_RATIO) / (Math.PI * WHEEL_DIAMETER_IN);

    // Strafing compensation for lateral under-travel (tune on your field surface)
    public static final double STRAFE_CORRECTION = DriveTuning.STRAFE_CORRECTION;

    // IMU turn control gains + tolerance
    public static final double TURN_KP = DriveTuning.TURN_KP;
    public static final double TURN_KD = DriveTuning.TURN_KD;
    public static final double TURN_TOLERANCE_DEG = DriveTuning.TURN_TOLERANCE_DEG;   // stop when within ±tuned tolerance
    public static final double TURN_SETTLE_TIME   = DriveTuning.TURN_SETTLE_TIME_SEC;  // remain within tolerance this many seconds

    // Auto translation taper floors (tunable)
    public static final double AUTO_MOVE_MIN_SPEED = DriveTuning.AUTO_MOVE_MIN_SPEED;
    public static final double AUTO_MOVE_WITH_TWIST_MIN_TRANS_SPEED = DriveTuning.AUTO_MOVE_WITH_TWIST_MIN_TRANS_SPEED;

    // ======= INTERNAL =======
    private final LinearOpMode linear;   // Non-null only in Autonomous usage
    private final Telemetry telemetry;

    private final DcMotorEx fl, fr, bl, br;
    private final IMU imu;

    // Remember baseline run mode to restore after RUN_TO_POSITION moves (Auto)
    private DcMotor.RunMode baseRunModeAfterMove = DcMotor.RunMode.RUN_USING_ENCODER;

    // ---------- Constructor for AUTONOMOUS (blocking helpers allowed) ----------
    // CHANGES (2025-11-04): Aligned Auto move() forward sign with TeleOp + added vector telemetry.
    // CHANGES (2025-11-04): stopAll() now reapplies BRAKE zero-power behavior before zeroing power.
    // CHANGES (2025-10-31): Added safeInit to guarantee zero drive power during INIT.
    // CHANGES (2025-11-27): Added moveWithTwist(...) to blend translation + heading change concurrently
    //                        for AutoSequence twist-enabled moves.
    // CHANGES (2025-11-29): Reworked move()/moveWithTwist(...) to stay in RUN_USING_ENCODER with
    //                        encoder-delta progress tracking and linear speed tapering for
    //                        speed-independent distance accuracy.
    // CHANGES (2025-11-29): Moved auto translation taper floors to DriveTuning so ramp minima can
    //                        be retuned without editing Drivebase.java.
    // CHANGES (2025-11-25): Exposed wheel encoder helpers for odometry consumers while
    //                        keeping TeleOp safe-init guarantees; refreshed changelog dates to match release.
    // CHANGES (2025-12-13): Reworked moveWithTwist to recompute the field-centric vector each loop so
    //                        translation stays aligned to the requested heading while twisting, and replaced
    //                        the fixed-direction tick target with encoder-derived translation accumulation to
    //                        keep distance termination accurate during turns.
    // CHANGES (2025-12-29): Restored encoder distance conversion to physical ticks so Auto distances
    //                        are no longer affected by odometry calibration.

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
        final double minSpeed = AUTO_MOVE_MIN_SPEED;
        double maxSpeed = clamp(speed, 0.1, 1.0);
        maxSpeed = Math.max(minSpeed, maxSpeed);

        double rad = toRadians(degrees);
        double forwardRaw = cos(rad);
        double lateralRaw = sin(rad);
        double forward = -forwardRaw;
        double lateral = lateralRaw;

        // Normalize vector so the speed cap applies uniformly across headings
        double vecNorm = max(1.0, max(abs(forward), abs(lateral)));
        forward /= vecNorm;
        lateral /= vecNorm;

        double xAdj = lateral * STRAFE_CORRECTION;
        double yAdj = forward;

        double flMult = yAdj + xAdj;
        double frMult = yAdj - xAdj;
        double blMult = yAdj - xAdj;
        double brMult = yAdj + xAdj;
        double multNorm = max(1.0, max(abs(flMult), max(abs(frMult), max(abs(blMult), abs(brMult)))));
        flMult /= multNorm; frMult /= multNorm; blMult /= multNorm; brMult /= multNorm;

        double baseTicks = distanceInches * TICKS_PER_IN;
        double targetTicks = (abs(flMult) + abs(frMult) + abs(blMult) + abs(brMult)) / 4.0 * abs(baseTicks);
        if (targetTicks < 5.0) {
            stopAll();
            return;
        }

        int flStart = fl.getCurrentPosition();
        int frStart = fr.getCurrentPosition();
        int blStart = bl.getCurrentPosition();
        int brStart = br.getCurrentPosition();

        while (isActive()) {
            double flDelta = abs(fl.getCurrentPosition() - flStart);
            double frDelta = abs(fr.getCurrentPosition() - frStart);
            double blDelta = abs(bl.getCurrentPosition() - blStart);
            double brDelta = abs(br.getCurrentPosition() - brStart);
            double meanDelta = (flDelta + frDelta + blDelta + brDelta) / 4.0;

            if (meanDelta >= targetTicks) {
                break;
            }

            double progress = clamp(meanDelta / targetTicks, 0.0, 1.0);
            double remaining = 1.0 - progress;
            double commandedSpeed = minSpeed + (maxSpeed - minSpeed) * remaining;
            commandedSpeed = clamp(commandedSpeed, minSpeed, maxSpeed);

            double driveCmd = commandedSpeed * forward;
            double strafeCmd = commandedSpeed * lateral;

            double flP = driveCmd + strafeCmd;
            double frP = driveCmd - strafeCmd;
            double blP = driveCmd - strafeCmd;
            double brP = driveCmd + strafeCmd;

            double maxMag = max(1.0, max(abs(flP), max(abs(frP), max(abs(blP), abs(brP)))));
            fl.setPower(flP / maxMag);
            fr.setPower(frP / maxMag);
            bl.setPower(blP / maxMag);
            br.setPower(brP / maxMag);

            idle();
        }

        stopAll();
    }

    /**
     * Field-centric translation while steering to a target heading.
     *
     * @param distanceInches     distance to move (inches)
     * @param degrees            heading of translation: 0=forward, +90=right, 180=back, -90=left
     * @param targetHeadingDeg   absolute heading to hold by the end of the move
     * @param translationSpeed   translation power cap (0..1)
     * @param twistSpeed         twist power cap (0..1)
     */
    public void moveWithTwist(double distanceInches,
                              double degrees,
                              double targetHeadingDeg,
                              double translationSpeed,
                              double twistSpeed) {
        if (!isActive()) return; // TeleOp: don't block

        final double minTranslationSpeed = AUTO_MOVE_WITH_TWIST_MIN_TRANS_SPEED;
        translationSpeed = clamp(translationSpeed, 0.1, 1.0);
        translationSpeed = Math.max(minTranslationSpeed, translationSpeed);
        twistSpeed = clamp(twistSpeed, 0.1, 1.0);

        double targetDistanceAbs = abs(distanceInches);
        if (targetDistanceAbs <= 1e-3) {
            stopAll();
            return;
        }

        int flLast = fl.getCurrentPosition();
        int frLast = fr.getCurrentPosition();
        int blLast = bl.getCurrentPosition();
        int brLast = br.getCurrentPosition();

        double traveledInches = 0.0;

        ElapsedTime dt = new ElapsedTime();
        double lastErr = shortestDiff(targetHeadingDeg, heading());

        while (isActive()) {
            double curHeading = heading();
            double fieldRelativeDeg = normDeg(degrees - curHeading);
            double rad = toRadians(fieldRelativeDeg);
            double forwardRaw = cos(rad);
            double lateralRaw = sin(rad);
            double forward = -forwardRaw;

            double maxVec = max(1.0, max(abs(forward), abs(lateralRaw)));
            forward /= maxVec;
            lateralRaw /= maxVec;

            double err = shortestDiff(targetHeadingDeg, curHeading);
            double derr = (err - lastErr) / max(1e-3, dt.seconds());
            lastErr = err; dt.reset();

            double twistCmd = TURN_KP * err + TURN_KD * derr;
            twistCmd = clamp(twistCmd, -twistSpeed, twistSpeed);

            int flPos = fl.getCurrentPosition();
            int frPos = fr.getCurrentPosition();
            int blPos = bl.getCurrentPosition();
            int brPos = br.getCurrentPosition();

            double flStep = flPos - flLast;
            double frStep = frPos - frLast;
            double blStep = blPos - blLast;
            double brStep = brPos - brLast;

            flLast = flPos;
            frLast = frPos;
            blLast = blPos;
            brLast = brPos;

            double driveStepTicks = (flStep + frStep + blStep + brStep) / 4.0;
            double strafeStepTicks = (flStep - frStep - blStep + brStep) / 4.0;

            double driveStepInches = driveStepTicks / TICKS_PER_IN;
            double strafeStepInches = strafeStepTicks / TICKS_PER_IN;
            traveledInches += abs(hypot(driveStepInches, strafeStepInches));

            double progress = clamp(traveledInches / targetDistanceAbs, 0.0, 1.0);
            double remaining = 1.0 - progress;
            double transSpeed = minTranslationSpeed
                    + (translationSpeed - minTranslationSpeed) * remaining;
            transSpeed = clamp(transSpeed, minTranslationSpeed, translationSpeed);

            double driveCmd = transSpeed * forward;
            double strafeCmd = transSpeed * lateralRaw;

            double flP = driveCmd + strafeCmd + twistCmd;
            double frP = driveCmd - strafeCmd - twistCmd;
            double blP = driveCmd - strafeCmd + twistCmd;
            double brP = driveCmd + strafeCmd - twistCmd;

            double maxMag = max(1.0, max(abs(flP), max(abs(frP), max(abs(blP), abs(brP)))));
            fl.setPower(flP / maxMag);
            fr.setPower(frP / maxMag);
            bl.setPower(blP / maxMag);
            br.setPower(brP / maxMag);

            if (traveledInches >= targetDistanceAbs) {
                break;
            }

            idle();
        }

        stopAll();

        // Finalize heading with the standard turn helper in case residual error remains.
        double residual = shortestDiff(targetHeadingDeg, heading());
        if (abs(residual) > TURN_TOLERANCE_DEG) {
            turn(residual, twistSpeed);
        }
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

    /** Current encoder counts for FL/FR/BL/BR in ticks (order matches motor names). */
    public int[] getWheelPositionsTicks() {
        return new int[]{
                fl.getCurrentPosition(),
                fr.getCurrentPosition(),
                bl.getCurrentPosition(),
                br.getCurrentPosition()
        };
    }

    /** Current wheel travel estimates in inches derived from encoder counts. */
    public double[] getWheelPositionsInches() {
        int[] ticks = getWheelPositionsTicks();
        return new double[]{
                ticks[0] / TICKS_PER_IN,
                ticks[1] / TICKS_PER_IN,
                ticks[2] / TICKS_PER_IN,
                ticks[3] / TICKS_PER_IN
        };
    }

    /** Immediately sets all four powers. */
    public void setAllPower(double p) {
        fl.setPower(p); fr.setPower(p); bl.setPower(p); br.setPower(p);
    }

    /** Stop all drive motors. (Alias retained for compatibility with TeleOp StopAll.) */
    public void stop() { stopAll(); }

    /** Stop all drive motors and ensure BRAKE is engaged. */
    public void stopAll() { applyBrakeHold(); }

    /** Engage BRAKE on all motors and zero power. */
    public void applyBrakeHold() {
        setAllZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setAllPower(0);
    }

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

    private void setAllZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) {
            if (m.getZeroPowerBehavior() != behavior) {
                m.setZeroPowerBehavior(behavior);
            }
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
