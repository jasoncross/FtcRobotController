package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/** Webcam AprilTag detection with an explicit library; no inherited game targets or poses. */
public final class AprilTagVision implements AutoCloseable {
    private final AprilTagProcessor processor;
    private final VisionPortal portal;

    /** Detect IDs and image centers only until measured tag metadata is supplied. */
    public AprilTagVision(HardwareMap hardwareMap) {
        this(hardwareMap, new AprilTagLibrary.Builder().build());
    }

    public AprilTagVision(HardwareMap hardwareMap, AprilTagLibrary tagLibrary) {
        processor = new AprilTagProcessor.Builder().setTagLibrary(tagLibrary).build();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, RobotConfig.WEBCAM_NAME))
                .setCameraResolution(new Size(RobotConfig.CAMERA_WIDTH, RobotConfig.CAMERA_HEIGHT))
                .addProcessor(processor)
                .build();
    }

    public List<AprilTagDetection> getDetections() {
        return new ArrayList<>(processor.getDetections());
    }

    public VisionPortal.CameraState getCameraState() {
        return portal.getCameraState();
    }

    @Override
    public void close() {
        portal.close();
    }
}
