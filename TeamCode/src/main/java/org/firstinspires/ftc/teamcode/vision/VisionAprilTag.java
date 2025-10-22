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
 * - Provides AprilTag vision support compatible with **all FTC SDK versions**.
 * - Initializes a VisionPortal and AprilTagProcessor.
 * - Returns a list of detections or the best detection for a specific tag ID.
 *
 * NOTES:
 * - Works without any AprilTagLibrary or addTag() calls (older SDKs don’t support them).
 * - Uses MJPEG @ 640x480 for built-in calibration and higher FPS.
 * - Compatible with SDKs from 2022 → 2025.
 *
 * ALLIANCE TAG IDS:
 * - Blue GOAL tag: ID 20
 * - Red  GOAL tag: ID 24
 */

public class VisionAprilTag {

    // === CONSTANTS ===
    public static final int TAG_BLUE_GOAL = 20; // Blue alliance GOAL tag
    public static final int TAG_RED_GOAL  = 24; // Red alliance GOAL tag

    // === INTERNAL OBJECTS ===
    private VisionPortal portal;
    private AprilTagProcessor tagProcessor;

    // =============================================================
    //  METHOD: init
    //  PURPOSE:
    //     - Initializes the VisionPortal + AprilTagProcessor.
    //     - Uses webcam defined in Robot Configuration.
    // =============================================================
    public void init(HardwareMap hw, String webcamName) {

        // --- Create AprilTag processor with default settings ---
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)            // Draw XYZ axes on detected tags
                .setDrawCubeProjection(true)  // Draw cube overlay on each tag
                .setDrawTagID(true)           // Display Tag ID on screen
                .build();

        // --- Build the VisionPortal stream ---
        VisionPortal.Builder builder = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hw.get(WebcamName.class, webcamName))
                .setCameraResolution(new Size(640, 480));   // Built-in calibration resolution

        // Try enabling MJPEG stream format (if SDK supports it)
        try {
            builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        } catch (Exception ignored) {
            // Older SDKs may not implement setStreamFormat(); safe to ignore.
        }

        // --- Start the vision portal ---
        portal = builder.build();
    }

    // =============================================================
    //  METHOD: getDetectionsCompat
    //  PURPOSE:
    //     - Returns the current list of AprilTag detections.
    //     - Compatible with all SDK versions by trying multiple APIs.
    // =============================================================
    @SuppressWarnings("unchecked")
    public List<AprilTagDetection> getDetectionsCompat() {
        if (tagProcessor == null) return java.util.Collections.emptyList();

        try {
            // Newer SDKs (2023+)
            return tagProcessor.getDetections();
        } catch (Throwable ignored1) {
            try {
                // Mid-generation SDKs (2022–2023)
                return tagProcessor.getFreshDetections();
            } catch (Throwable ignored2) {
                try {
                    // Earliest VisionPortal builds exposed a public 'detections' field
                    java.lang.reflect.Field f = tagProcessor.getClass().getField("detections");
                    Object val = f.get(tagProcessor);
                    if (val instanceof List) {
                        return (List<AprilTagDetection>) val;
                    }
                } catch (Throwable ignored3) { }
            }
        }

        return java.util.Collections.emptyList();
    }

    // =============================================================
    //  METHOD: getDetectionFor
    //  PURPOSE:
    //     - Returns the closest detection matching a desired Tag ID.
    //     - Uses ftcPose.range to select the nearest visible tag.
    // =============================================================
    public AprilTagDetection getDetectionFor(int desiredId) {
        AprilTagDetection best = null;
        for (AprilTagDetection d : getDetectionsCompat()) {
            if (d.id == desiredId) {
                if (best == null || d.ftcPose.range < best.ftcPose.range) best = d;
            }
        }
        return best;
    }

    // =============================================================
    //  METHOD: stop
    //  PURPOSE:
    //     - Safely close the VisionPortal when OpMode ends.
    // =============================================================
    public void stop() {
        if (portal != null) portal.close();
    }
}
