package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.config.ControllerTuning;
import org.firstinspires.ftc.teamcode.config.TeleOpRumbleTuning;
import org.firstinspires.ftc.teamcode.config.VisionConfig;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.input.ControllerBindings;
import org.firstinspires.ftc.teamcode.utils.RumbleNotifier;
import org.firstinspires.ftc.teamcode.vision.VisionFactory;
import org.firstinspires.ftc.teamcode.vision.VisionTargetProvider;
import org.firstinspires.ftc.teamcode.vision.TargetObservation;
import org.firstinspires.ftc.teamcode.vision.TagAimController;

@TeleOp(name = "BIOBUZZ: Base Drive", group = "BIOBUZZ")
@Disabled // Review RobotConfig and test wheel directions before enabling.
public class BaseDriveTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        VisionTargetProvider vision = null;
        TagAimController aim = new TagAimController();
        RumbleNotifier rumble = new RumbleNotifier(gamepad1);
        rumble.setActive(TeleOpRumbleTuning.AIM_RUMBLE_ENABLED);
        rumble.setThresholdDeg(TeleOpRumbleTuning.AIM_THRESHOLD_DEG);
        rumble.setMinMax(TeleOpRumbleTuning.AIM_STRENGTH_MIN, TeleOpRumbleTuning.AIM_STRENGTH_MAX,
                TeleOpRumbleTuning.AIM_PULSE_MIN_MS, TeleOpRumbleTuning.AIM_PULSE_MAX_MS,
                TeleOpRumbleTuning.AIM_COOLDOWN_MIN_MS, TeleOpRumbleTuning.AIM_COOLDOWN_MAX_MS);
        ControllerBindings bindings = new ControllerBindings();
        bindings.bindPress(ControllerBindings.Pad.G1, ControllerBindings.Btn.Y, () -> {
            rumble.setActive(!rumble.isActive());
            gamepad1.runRumbleEffect(new com.qualcomm.robotcore.hardware.Gamepad.RumbleEffect.Builder()
                    .addStep(TeleOpRumbleTuning.TOGGLE_STRENGTH, TeleOpRumbleTuning.TOGGLE_STRENGTH,
                            TeleOpRumbleTuning.TOGGLE_STEP_MS)
                    .addStep(0, 0, TeleOpRumbleTuning.TOGGLE_GAP_MS)
                    .addStep(TeleOpRumbleTuning.TOGGLE_STRENGTH, TeleOpRumbleTuning.TOGGLE_STRENGTH,
                            TeleOpRumbleTuning.TOGGLE_STEP_MS).build());
        });
        try {
            if (RobotConfig.VISION_ENABLED) vision = VisionFactory.create(hardwareMap);
            telemetry.addLine("Left stick: move; right X: turn; LT: slow; A: stop; Y: rumble toggle.");
            telemetry.addLine("Hold RB: aim only when explicitly enabled and a target ID is configured.");
            telemetry.addData("Drive power limit", RobotConfig.DRIVE_POWER_LIMIT);
            telemetry.update();
            while (opModeInInit()) {
                if (vision != null) { vision.update(); telemetry.addData("Vision", vision.getStatus()); }
                telemetry.update();
                sleep(20);
            }
            waitForStart();
            while (opModeIsActive()) {
                bindings.update(gamepad1, gamepad2);
                if (vision != null) vision.update();
                TargetObservation target = vision == null ? null : vision.getTarget(VisionConfig.TARGET_TAG_ID);
                double twist = deadband(gamepad1.right_stick_x) * ControllerTuning.TURN_SCALE;
                if (VisionConfig.ENABLE_AIM_ASSIST && gamepad1.right_bumper && !gamepad1.a) {
                    // No stale-heading hold: loss of the selected target yields zero aim twist.
                    twist = aim.turnPower(target, System.nanoTime());
                } else {
                    aim.reset();
                }
                double floor = Math.max(0, Math.min(1, ControllerTuning.SLOWEST_SPEED));
                double scale = 1 - Math.max(0, Math.min(1, gamepad1.left_trigger)) * (1 - floor);
                if (gamepad1.a) {
                    drive.stop();
                } else {
                    drive.drive(deadband(-gamepad1.left_stick_y),
                            deadband(gamepad1.left_stick_x) * ControllerTuning.STRAFE_SCALE, twist, scale);
                }
                if (vision != null) {
                    telemetry.addData("Vision", vision.getStatus());
                    telemetry.addData("Tags detected", vision.getTargets().size());
                }
                if (target != null && target.isFresh(System.nanoTime(), VisionConfig.MAX_TARGET_AGE_MS))
                    rumble.update(target.bearingDeg - VisionConfig.AIM_BEARING_OFFSET_DEG);
                telemetry.addData("Drive", gamepad1.a ? "Stopped" : "Robot-centric");
                telemetry.update();
                idle();
            }
        } finally {
            drive.stop();
            bindings.clear();
            gamepad1.stopRumble();
            if (vision != null) vision.close();
        }
    }

    private static double deadband(double value) {
        return Math.abs(value) < ControllerTuning.STICK_DEADBAND ? 0 : value;
    }
}
