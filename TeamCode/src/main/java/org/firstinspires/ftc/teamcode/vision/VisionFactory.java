package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.VisionConfig;

public final class VisionFactory {
    private VisionFactory() { }
    public static VisionTargetProvider create(HardwareMap hardwareMap) {
        return VisionConfig.SOURCE == VisionConfig.Source.LIMELIGHT
                ? new LimelightTargetProvider(hardwareMap) : new AprilTagVision(hardwareMap);
    }
}
