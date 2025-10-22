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
 * - Owns VisionPortal + AprilTagProcessor for AprilTag detection.
 * - Provides helper to get the "best" (closest) detection for a specific tag id.
 * - Used by TeleOpAllianceBase to implement Aim-Assist (twist only).
 *
 * ALLIANCE GOAL TAGS:
 * - BLUE: 20
 * - RED : 24
 *
 * CAMERA / STREAM:
 * - We request MJPEG @ 640x480 for higher FPS and to use SDK built-in pose calibration.
 * - If you change to 1280x720 (or other), expect calibration warning unless adding intrinsics.
 *
 * METHODS:
 * - init(hw, webcamName):    create VisionPortal + AprilTagProcessor.
 * - getDetectionFor(id):     return closest detection for a given id (or null).
 * - stop():                  close the portal cleanly on OpMode stop().
 *
 * NOTES:
 * - The webcam name must match the Robot Configuration (default: "Webcam 1").
 */

public class VisionAprilTag {

    // ---------------- Constants ----------------
    public static final int TAG_BLUE_GOAL = 20; // Alliance GOAL tag (Blue)
    public static final int TAG_RED_GOAL  = 24; // Alliance GOAL tag (Red)

    // ---------------- Members ----------------
    private VisionPortal portal;
    private AprilTagProcessor tagProcessor;

    /**
     * Initialize the camera + AprilTag processor.
     * @param hw          HardwareMap
     * @param webcamName  Must match Robot Configuration (e.g., "Webcam 1")
     */
    public void init(HardwareMap hw, String webcamName) {
        // ==============================================================
        // APRILTAG PROCESSOR
        // ==============================================================
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .build();

        // ==============================================================
        // VISION PORTAL (Camera + Stream)
        // - Use 640x480 (built-in calibration) and MJPEG for higher FPS.
        // ==============================================================
        portal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hw.get(WebcamName.class, webcamName))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();
    }

    /**
     * Return the closest detection for the desired tag id, or null if none are visible.
     * "Closest" = smallest ftcPose.range (meters).
     * @param desiredId AprilTag id to search for
     * @return AprilTagDetection or null
     */
    public AprilTagDetection getDetectionFor(int desiredId) {
        if (tagProcessor == null) return null;

        List<AprilTagDetection> detections = tagProcessor.getDetections();
        AprilTagDetection best = null;

        for (AprilTagDetection d : detections) {
            if (d.id == desiredId) {
                if (best == null || d.ftcPose.range < best.ftcPose.range) {
                    best = d;
                }
            }
        }
        return best;
    }

    /**
     * Close the VisionPortal and free camera resources.
     * Safe to call multiple times or when portal is null.
     */
    public void stop() {
        if (portal != null) {
            portal.close();
            portal = null;
        }
    }
}
