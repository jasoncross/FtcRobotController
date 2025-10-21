package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.*;

/*
 * FILE: Drivebase.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/
 *
 * PURPOSE:
 * - Centralized, reusable mecanum drive library used by both TeleOp and Autonomous.
 * - Exposes three main controls:
 *     1) drive(drive, strafe, twist)           -> manual robot-centric driving
 *     2) move(distanceInches, degrees, speed)  -> encoder-based translation (no turning)
 *     3) turn(degrees, speed)                  -> IMU-based in-place rotation
 * - Initializes IMU orientation correctly and sets up motors/encoders/zero-power behaviors.
 *
 * REQUIRED MOTOR NAMES (Robot Configuration must match EXACTLY):
 * - "FrontLeft", "FrontRight", "BackLeft", "BackRight"
 * - IMU name: "imu"
 *
 * TUNABLES (WHAT YOU MAY NEED TO CHANGE & WHY):
 * - WHEEL_DIAMETER_IN:    (inches) Real measured wheel diameter. Affects move() distance accuracy.
 * - TICKS_PER_REV:        Encoder ticks per motor shaft revolution. (e.g., goBILDA 5202 312RPM = 537.7)
 * - GEAR_RATIO:           If you have external gearing between motor and wheel. (>1 if wheel turns slower than motor)
 * - STRAFE_CORRECTION:    Strafing usually under-travels with mecanum; >1.0 increases lateral distance.
 *                         Tune by commanding move(24, 90, ...) and measuring actual distance; adjust until it’s 24".
 * - TURN_KP, TURN_KD:     Proportional and derivative gains for IMU turns. Increase KP for stronger correction;
 *                         add KD to reduce overshoot. Tune to reach target quickly without oscillation.
 * - TURN_TOLERANCE_DEG:   How close (in degrees) turn() must get before considering "on target".
 * - TURN_SETTLE_TIME:     Time (seconds) within tolerance before declaring the turn complete (prevents jittery exits).
 *
 * IMPORTANT CLASSES/FUNCTIONS:
 * - Drivebase(LinearOpMode op): Constructor that wires hardware & IMU.
 * - void drive(double drive, double strafe, double twist):
 *      Robot-centric power computation with normalization. Call every loop in TeleOp.
 * - void move(double distanceInches, double degrees, double speed):
 *      Encoder RUN_TO_POSITION move; 0°=forward, +90°=right, 180°=back, -90°=left.
 *      Uses STRAFE_CORRECTION to improve lateral accuracy; does not rotate.
 * - void turn(double degrees, double speed):
 *      IMU-based relative rotation; +degrees = CCW. Uses simple PD with tolerance/settle.
 * - void resetHeading(), double heading():
 *      Utilities to zero and read the robot’s yaw in degrees (CCW positive).
 *
 * NOTES:
 * - Motor directions below assume a typical mecanum layout. If your robot drives "backwards"
 *   or strafes the wrong way, flip motor directions as needed (start with FR/BR).
 * - All drivetrain motors are set to BRAKE for snappy stopping & precise auto behavior.
 * - This class expects a LinearOpMode for access to opModeIsActive()/idle() during blocking moves/turns.
 */

public class Drivebase {
    // ======= TUNE THESE CONSTANTS FOR YOUR ROBOT =======
    public static final double WHEEL_DIAMETER_IN = 3.7795; // 96 mm goBILDA wheel ≈ 3.7795"
    public static final double TICKS_PER_REV     = 537.7;  // goBILDA 5202 (312RPM) encoder
    public static final double GEAR_RATIO        = 1.0;    // wheel revs per motor rev (use >1.0 if geared down)
    public static final double TICKS_PER_IN      = (TICKS_PER_REV * GEAR_RATIO) / (Math.PI * WHEEL_DIAMETER_IN);

    // Strafing compensation (lateral under-travel). Start 1.10–1.25 and tune on your carpet/tiles.
    public static final double STRAFE_CORRECTION = 1.15;

    // IMU turn control gains + tolerance
    public static final double TURN_KP = 0.012;
    public static final double TURN_KD = 0.003;
    public static final double TURN_TOLERANCE_DEG = 1.0; // stop when within ±1°
    public static final double TURN_SETTLE_TIME   = 0.15; // stay within tolerance for 0.15s

    // ======= INTERNAL STATE =======
    private final LinearOpMode op;
    private final DcMotorEx fl, fr, bl, br;
    private final IMU imu;

    public Drivebase(LinearOpMode op) {
        this.op = op;

        // --- Motors ---
        fl = op.hardwareMap.get(DcMotorEx.class, "FrontLeft");
        fr = op.hardwareMap.get(DcMotorEx.class, "FrontRight");
        bl = op.hardwareMap.get(DcMotorEx.class, "BackLeft");
        br = op.hardwareMap.get(DcMotorEx.class, "BackRight");

        // Standard mecanum directions (flip if needed on-bot)
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // --- IMU (ensure hub orientation matches your mounting) ---
        imu = op.hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));
        resetHeading();
    }

    /** Robot-centric mecanum drive. Call every TeleOp loop. */
    public void drive(double drive, double strafe, double twist) {
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

    /**
     * Encoder-based translation without rotation.
     * @param distanceInches distance to move (inches)
     * @param degrees heading of translation: 0=forward, +90=right, 180=back, -90=left
     * @param speed power cap 0..1 (use 0.3–0.7 for accurate autos)
     */
    public void move(double distanceInches, double degrees, double speed) {
        speed = clamp(speed, 0.1, 1.0);

        // Polar to robot-centric (x=right, y=forward) with 0°=forward convention
        double rad = toRadians(degrees);
        double x =  cos(rad); // right
        double y =  sin(rad); // forward

        // Compensate strafing losses on x
        double xAdj = x * STRAFE_CORRECTION;
        double yAdj = y;

        // No twist component for pure translation
        double flMult = yAdj + xAdj;
        double frMult = yAdj - xAdj;
        double blMult = yAdj - xAdj;
        double brMult = yAdj + xAdj;

        // Normalize wheel multipliers
        double maxMag = max(1.0, max(abs(flMult), max(abs(frMult), max(abs(blMult), abs(brMult)))));
        flMult /= maxMag; frMult /= maxMag; blMult /= maxMag; brMult /= maxMag;

        int baseTicks = (int) round(distanceInches * TICKS_PER_IN);
        int flT = fl.getCurrentPosition() + (int) round(baseTicks * flMult);
        int frT = fr.getCurrentPosition() + (int) round(baseTicks * frMult);
        int blT = bl.getCurrentPosition() + (int) round(baseTicks * blMult);
        int brT = br.getCurrentPosition() + (int) round(baseTicks * brMult);

        setRunToPosition(flT, frT, blT, brT);
        setAllPower(speed);

        while (op.opModeIsActive() && (fl.isBusy() || fr.isBusy() || bl.isBusy() || br.isBusy())) {
            op.idle(); // yield to SDK
        }
        stopAndUseEncoder();
    }

    /**
     * IMU-based in-place rotation (relative).
     * @param degrees +CCW / -CW
     * @param speed power cap 0..1 (0.3–0.6 typical)
     */
    public void turn(double degrees, double speed) {
        speed = clamp(speed, 0.2, 1.0);
        double start = heading();
        double target = normDeg(start + degrees);

        ElapsedTime dt = new ElapsedTime();
        double lastErr = 0, settleStart = -1;

        while (op.opModeIsActive()) {
            double cur = heading();
            double err = shortestDiff(target, cur);
            double derr = (err - lastErr) / max(1e-3, dt.seconds());
            lastErr = err; dt.reset();

            double cmd = TURN_KP * err + TURN_KD * derr;
            cmd = clamp(cmd, -speed, speed);

            fl.setPower(cmd); bl.setPower(cmd);
            fr.setPower(-cmd); br.setPower(-cmd);

            // Exit when within tolerance and stable
            if (abs(err) <= TURN_TOLERANCE_DEG) {
                if (settleStart < 0) settleStart = System.nanoTime()/1e9;
                if ((System.nanoTime()/1e9 - settleStart) >= TURN_SETTLE_TIME) break;
            } else {
                settleStart = -1;
            }
            op.idle();
        }
        stopAll();
    }

    // ----------- Helpers (internal) -----------
    private void setRunToPosition(int flT, int frT, int blT, int brT) {
        fl.setTargetPosition(flT); fr.setTargetPosition(frT);
        bl.setTargetPosition(blT); br.setTargetPosition(brT);
        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) {
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    private void stopAndUseEncoder() {
        stopAll();
        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void setAllPower(double p) { fl.setPower(p); fr.setPower(p); bl.setPower(p); br.setPower(p); }
    public  void stopAll() { setAllPower(0); }

    /** Zero the robot's yaw so heading() returns ~0 from this moment forward. */
    public void resetHeading() { imu.resetYaw(); }

    /** Current yaw in DEGREES, CCW positive. */
    public double heading() {
        return imu.getRobotYawPitchRollAngles()
                  .getYaw(RevHubOrientationOnRobot.AngleUnit.DEGREES);
    }

    private static double shortestDiff(double target, double current) {
        double d = normDeg(target - current);
        if (d > 180) d -= 360; if (d < -180) d += 360; return d;
    }
    private static double normDeg(double a) { double r = a % 360; if (r < 0) r += 360; return r; }
    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
}
