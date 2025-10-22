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

public class Drivebase {
    // ======= TUNE THESE =======
    public static final double WHEEL_DIAMETER_IN = 3.7795;
    public static final double TICKS_PER_REV     = 537.7;
    public static final double GEAR_RATIO        = 1.0;
    public static final double TICKS_PER_IN      = (TICKS_PER_REV * GEAR_RATIO) / (Math.PI * WHEEL_DIAMETER_IN);
    public static final double STRAFE_CORRECTION = 1.15;

    public static final double TURN_KP = 0.012;
    public static final double TURN_KD = 0.003;
    public static final double TURN_TOLERANCE_DEG = 1.0;
    public static final double TURN_SETTLE_TIME   = 0.15;

    // ======= INTERNAL =======
    private final LinearOpMode linear;       // null in TeleOp
    private final Telemetry telemetry;
    private final DcMotorEx fl, fr, bl, br;
    private final IMU imu;

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
        commonInit();
    }

    // ---------- Constructor for TELEOP (no blocking helpers) ----------
    public Drivebase(HardwareMap hw, Telemetry telemetry) {
        this.linear = null; // TeleOp
        this.telemetry = telemetry;
        imu  = hw.get(IMU.class, "imu");
        fl   = hw.get(DcMotorEx.class, "FrontLeft");
        fr   = hw.get(DcMotorEx.class, "FrontRight");
        bl   = hw.get(DcMotorEx.class, "BackLeft");
        br   = hw.get(DcMotorEx.class, "BackRight");
        commonInit();
    }

    private void commonInit() {
        // Standard mecanum directions (flip FR/BR if your robot goes backwards)
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));
        resetHeading();
    }

    // -------- TeleOp drive (robot-centric) --------
    public void drive(double drive, double strafe, double twist) {
        double flP = drive + strafe + twist;
        double frP = drive - strafe - twist;
        double blP = drive - strafe + twist;
        double brP = drive + strafe - twist;

        double maxMag = max(1.0, max(abs(flP), max(abs(frP), max(abs(blP), abs(brP)))));
        fl.setPower(flP / maxMag); fr.setPower(frP / maxMag); bl.setPower(blP / maxMag); br.setPower(brP / maxMag);
    }

    // -------- Encoder translation (no turning) --------
    public void move(double distanceInches, double degrees, double speed) {
        if (!isActive()) {
            // In TeleOp we don't block; just return to avoid hanging
            return;
        }

        speed = clamp(speed, 0.1, 1.0);

        double rad = toRadians(degrees);
        double x =  cos(rad); // right
        double y =  sin(rad); // forward

        double xAdj = x * STRAFE_CORRECTION;
        double yAdj = y;

        double flMult = yAdj + xAdj;
        double frMult = yAdj - xAdj;
        double blMult = yAdj - xAdj;
        double brMult = yAdj + xAdj;

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
            // optional telemetry here
            idle();
        }
        stopAndUseEncoder();
    }

    // -------- IMU-based turn --------
    public void turn(double degrees, double speed) {
        if (!isActive()) {
            // TeleOp: don't block
            return;
        }

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
                if (settleStart < 0) settleStart = System.nanoTime()/1e9;
                if ((System.nanoTime()/1e9 - settleStart) >= TURN_SETTLE_TIME) break;
            } else settleStart = -1;

            idle();
        }
        stopAll();
    }

    // ---------- helpers ----------
    private void setRunToPosition(int flT, int frT, int blT, int brT) {
        fl.setTargetPosition(flT); fr.setTargetPosition(frT);
        bl.setTargetPosition(blT); br.setTargetPosition(brT);
        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void stopAndUseEncoder() {
        stopAll();
        for (DcMotorEx m : new DcMotorEx[]{fl, fr, bl, br}) m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setAllPower(double p) { fl.setPower(p); fr.setPower(p); bl.setPower(p); br.setPower(p); }
    public  void stopAll() { setAllPower(0); }

    public void resetHeading() { imu.resetYaw(); }

    /** Current yaw degrees, CCW positive. */
    public double heading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES); // ✅ fixed
    }

    private boolean isActive() { return linear != null && linear.opModeIsActive(); }
    private void idle() { if (linear != null) linear.idle(); }

    private static double shortestDiff(double target, double current) {
        double d = normDeg(target - current);
        if (d > 180) d -= 360; if (d < -180) d += 360; return d;
    }
    private static double normDeg(double a) { double r = a % 360; if (r < 0) r += 360; return r; }
    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
}
