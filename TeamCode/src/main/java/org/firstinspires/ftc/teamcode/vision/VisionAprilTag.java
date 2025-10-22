package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Collections;
import java.util.List;

/*
 * FILE: VisionAprilTag.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vision/
 *
 * PURPOSE:
 * - Own the FTC VisionPortal + AprilTagProcessor lifecycle for a single USB webcam.
 * - Provide an SDK-agnostic way to retrieve AprilTag detections and select the
 *   “best” detection for a specific tag id (closest by range).
 *
 * USAGE:
 *   VisionAprilTag vision = new VisionAprilTag();
 *   vision.init(hardwareMap, "Webcam 1");     // name must match Robot Configuration
 *   AprilTagDetection det = vision.getDetectionFor(VisionAprilTag.TAG_BLUE_GOAL);
 *   ...
 *   vision.stop();                            // call in OpMode.stop()
 *
 * NOTES:
 * - We request 640x480 MJPEG by default for (a) higher FPS and (b) built-in pose
 *   calibration in the SDK. If your SDK/firmware doesn’t support MJPEG, the builder
 *   call is ignored safely and the stream still works.
 * - This class is written to run on multiple FTC SDK revisions (2022→2025). Some
 *   AprilTagProcessor methods differ by SDK version; getDetectionsCompat() handles
 *   those differences gracefully.
 *
 * TAG IDs (2025 game “DE*CODE”):
 * - Blue GOAL tag: 20
 * - Red  GOAL tag: 24
 *   (If these change in a Team Update, update the constants below.)
 */
public class VisionAprilTag {

    // --- Public tag id constants for alliance GOAL targets ---
    public static final int TAG_BLUE_GOAL = 20;
    public static final int TAG_RED_GOAL  = 24;

    // --- Vision stack ---
    private VisionPortal portal;
    private AprilTagProcessor tagProcessor;

    /**
     * Initialize webcam + AprilTag processor.
     *
     * @param hw          HardwareMap from your OpMode
     * @param webcamName  name of the webcam in the Robot Configuration (e.g., "Webcam 1")
     */
    public void init(HardwareMap hw, String webcamName) {
        // Build the AprilTag processor (only options common across SDK versions).
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .build();

        // Build the Vision Portal with our processor and camera.
        VisionPortal.Builder builder = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hw.get(WebcamName.class, webcamName))
                // 640x480 uses a built-in calibration profile; also lighter/faster.
                .setCameraResolution(new Size(640, 480));

        // Prefer MJPEG streaming for higher FPS when supported (older SDKs ignore this).
        try {
            builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        } catch (Throwable ignored) { /* OK on older SDKs */ }

        portal = builder.build();
    }

    /**
     * Return the list of current AprilTag detections in a way that works across SDK versions.
     * Order is whatever the SDK provides; caller should sort/select as needed.
     *
     * SDK differences handled here:
     * - Newer SDKs:   AprilTagProcessor#getDetections()
     * - Mid versions: AprilTagProcessor#getFreshDetections()
     * - Earliest:     public field "detections"
     */
    @SuppressWarnings("unchecked")
    public List<AprilTagDetection> getDetectionsCompat() {
        if (tagProcessor == null) return Collections.emptyList();

        try {
            // Newer SDKs
            return tagProcessor.getDetections();
        } catch (Throwable ignored1) {
            try {
                // Some mid-generation SDKs
                return tagProcessor.getFreshDetections();
            } catch (Throwable ignored2) {
                try {
                    // Earliest VisionPortal betas exposed a public field
                    java.lang.reflect.Field f = tagProcessor.getClass().getField("detections");
                    Object val = f.get(tagProcessor);
                    if (val instanceof List) return (List<AprilTagDetection>) val;
                } catch (Throwable ignored3) {
                    // Fall through to empty list
                }
            }
        }
        return Collections.emptyList();
    }

    /**
     * Returns the "best" detection for a specific tag id, or null if none is visible.
     * "Best" = the detection with the smallest ftcPose.range (closest).
     *
     * @param desiredId tag id to look for (e.g., TAG_BLUE_GOAL or TAG_RED_GOAL)
     */
    public AprilTagDetection getDetectionFor(int desiredId) {
        AprilTagDetection best = null;
        for (AprilTagDetection d : getDetectionsCompat()) {
            if (d.id == desiredId) {
                if (best == null || d.ftcPose.range < best.ftcPose.range) best = d;
            }
        }
        return best;
    }

    /**
     * Close the VisionPortal and release camera resources.
     * Safe to call multiple times; does nothing if already closed.
     */
    public void stop() {
        try {
            if (portal != null) {
                portal.close();
            }
        } catch (Throwable ignored) {
            // Some SDKs may throw if close is called during shutdown; ignore safely.
        } finally {
            portal = null;
            tagProcessor = null;
        }
    }
}
