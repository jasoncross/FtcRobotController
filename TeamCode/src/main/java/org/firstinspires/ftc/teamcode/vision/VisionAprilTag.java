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
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vision/
 *
 * PURPOSE:
 * - Owns VisionPortal + AprilTagProcessor and provides the best detection
 *   for a specific tag ID (used for alliance GOAL targeting).
 *
 * IMPORTANT:
 * - Add your USB camera to the Robot Configuration as "Webcam 1" (or rename
 *   and update the init call).
 *
 * TUNABLES:
 * - Camera resolution: 1280x720 is a good balance for AprilTags.
 * - Draw options: enable/disable for debugging overlays.
 */
public class VisionAprilTag {
    public static final int TAG_BLUE_GOAL = 20; // Alliance GOAL tag (Blue)
    public static final int TAG_RED_GOAL  = 24; // Alliance GOAL tag (Red)

    private VisionPortal portal;
    private AprilTagProcessor tagProcessor;

    /** Initialize camera + AprilTag processor. webcamName must match Robot Config. */
    public void init(HardwareMap hw, String webcamName) {
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .build();

        portal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hw.get(WebcamName.class, webcamName))
                .setCameraResolution(new Size(1280, 720))
                .build();
    }

    /** Returns the closest detection for the given tag id, or null if none visible. */
    public AprilTagDetection getDetectionFor(int desiredId) {
        if (tagProcessor == null) return null;
        List<AprilTagDetection> detections = tagProcessor.getDetections();
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
