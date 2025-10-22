package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/*
 * FILE: VisionAprilTag.java
 * PURPOSE:
 * - Initialize VisionPortal and AprilTagProcessor.
 * - Return the best detection for the selected tag ID.
 *
 * NOTES:
 * - Uses 640x480 MJPEG for higher FPS and built-in calibration (no SDK warnings).
 * - Compatible with older FTC SDKs that don't include setDecimation(), setGainRange(), or setLensIntrinsics().
 */
public class VisionAprilTag {
    public static final int TAG_BLUE_GOAL = 20; // Blue alliance GOAL tag
    public static final int TAG_RED_GOAL  = 24; // Red alliance GOAL tag

    private VisionPortal portal;
    private AprilTagProcessor tagProcessor;

    /** Initialize camera + AprilTag processor. webcamName must match Robot Config. */
    public void init(HardwareMap hw, String webcamName) {
        // Build the AprilTag processor with defaults (no optional methods)
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .build();

        // Build the Vision Portal stream
        VisionPortal.Builder builder = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hw.get(WebcamName.class, webcamName))
                .setCameraResolution(new Size(640, 480));

        // Use MJPEG if supported; older SDKs ignore this silently
        try {
            builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        } catch (Exception ignored) { }

        portal = builder.build();
    }

    /** Returns the closest detection for the given tag id, or null if none visible. */
    public AprilTagDetection getDetectionFor(int desiredId) {
    if (tagProcessor == null) return null;
    List<AprilTagDetection> detections = null;

    try {
        // Works on SDKs that have getDetections()
        detections = tagProcessor.getDetections();
    } catch (Exception e1) {
        try {
            // Works on older SDKs with getFreshDetections()
            detections = tagProcessor.getFreshDetections();
        } catch (Exception e2) {
            try {
                // Works on earliest SDKs exposing a public field 'detections'
                java.lang.reflect.Field f = tagProcessor.getClass().getField("detections");
                Object val = f.get(tagProcessor);
                if (val instanceof List) detections = (List<AprilTagDetection>) val;
            } catch (Exception ignored) { }
        }
    }

    if (detections == null || detections.isEmpty()) return null;

    AprilTagDetection best = null;
    for (AprilTagDetection d : detections) {
        if (d.id == desiredId) {
            if (best == null || d.ftcPose.range < best.ftcPose.range) best = d;
        }
    }
    return best;
}


    public void stop() {
        if (portal != null) portal.close();
    }
}
