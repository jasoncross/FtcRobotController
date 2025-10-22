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
 *   Cross-SDK AprilTag helper that:
 *     - Initializes VisionPortal + AprilTagProcessor (SDK-neutral).
 *     - Provides a compatibility getter for detections across SDK versions.
 *     - Returns the "best" detection for a specific Tag ID (closest range).
 *     - Adds a configurable RANGE SCALE to correct distance if pose is off
 *       by a constant factor (common with older SDKs or uncalibrated cams).
 *
 * NOTES:
 *   - We use 640x480 with MJPEG when available (built-in calibration + better FPS).
 *   - No AprilTagLibrary/addTag() calls so this compiles on older SDKs.
 *   - Bearing/pose are taken from AprilTagDetection.ftcPose.
 *
 * ALLIANCE TAG IDS (DECODE field):
 *   - BLUE GOAL: 20
 *   - RED  GOAL: 24
 *
 * TUNING:
 *   - Call setRangeScale(s) after a 1-point calibration:
 *       s = true_distance_meters / measured_distance_meters
 *     Then use getScaledRange(det) anywhere you display/use distance.
 *
 * METHODS:
 *   - init(hw, "Webcam 1")
 *   - List<AprilTagDetection> getDetectionsCompat()
 *   - AprilTagDetection getDetectionFor(int tagId)
 *   - void   setRangeScale(double s)
 *   - double getScaledRange(AprilTagDetection det)
 *   - void   stop()
 */
public class VisionAprilTag {

    // === CONSTANTS ===
    public static final int TAG_BLUE_GOAL = 20; // Blue alliance GOAL tag
    public static final int TAG_RED_GOAL  = 24; // Red alliance GOAL tag

    // === INTERNAL OBJECTS ===
    private VisionPortal portal;
    private AprilTagProcessor tagProcessor;

    // === RANGE SCALING ===
    // Multiply raw ftcPose.range (meters) by this factor to correct distance.
    // Set via setRangeScale() after quick tape-measure calibration.
    private double rangeScale = 1.0;

    // =============================================================
    //  METHOD: init
    //  PURPOSE:
    //     - Initialize the VisionPortal + AprilTagProcessor.
    //     - Uses webcam defined in Robot Configuration.
    // =============================================================
    public void init(HardwareMap hw, String webcamName) {
        // Build AprilTag processor with widely supported options
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)            // Draw XYZ axes on detected tags
                .setDrawCubeProjection(true)  // Draw cube overlay on each tag
                .setDrawTagID(true)           // Display Tag ID on stream
                .build();

        // Build VisionPortal
        VisionPortal.Builder builder = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hw.get(WebcamName.class, webcamName))
                .setCameraResolution(new Size(640, 480));   // Built-in calibration res

        // Prefer MJPEG for higher FPS (older SDKs may not support; ignore if so)
        try { builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG); } catch (Exception ignored) {}

        portal = builder.build();
    }

    // =============================================================
    //  METHOD: getDetectionsCompat
    //  PURPOSE:
    //     - Return a list of detections regardless of SDK version.
    //     - Tries multiple APIs, then falls back to a public field.
    // =============================================================
    @SuppressWarnings("unchecked")
    public List<AprilTagDetection> getDetectionsCompat() {
        if (tagProcessor == null) return java.util.Collections.emptyList();

        try {
            // Newer SDKs
            return tagProcessor.getDetections();
        } catch (Throwable ignored1) {
            try {
                // Mid-generation SDKs
                return tagProcessor.getFreshDetections();
            } catch (Throwable ignored2) {
                try {
                    // Earliest VisionPortal builds: public field "detections"
                    java.lang.reflect.Field f = tagProcessor.getClass().getField("detections");
                    Object val = f.get(tagProcessor);
                    if (val instanceof List) {
                        return (List<AprilTagDetection>) val;
                    }
                } catch (Throwable ignored3) { /* fall through */ }
            }
        }
        return java.util.Collections.emptyList();
    }

    // =============================================================
    //  METHOD: getDetectionFor
    //  PURPOSE:
    //     - Return the closest visible detection matching the given Tag ID.
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
    //  METHOD: setRangeScale / getScaledRange
    //  PURPOSE:
    //     - Apply a constant multiplier to correct distance output.
    //     - Use getScaledRange(det) anywhere you display/use distance.
    // =============================================================
    public void setRangeScale(double s) { this.rangeScale = s; }

    public double getScaledRange(AprilTagDetection det) {
        return (det == null) ? Double.NaN : det.ftcPose.range * rangeScale;
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
