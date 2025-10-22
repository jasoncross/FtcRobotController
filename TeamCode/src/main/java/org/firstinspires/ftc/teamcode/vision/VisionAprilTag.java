package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;

import java.util.Collections;
import java.util.List;

/*
 * FILE: VisionAprilTag.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vision/
 *
 * PURPOSE:
 * - Provide all camera + AprilTag vision handling for the robot.
 * - Owns the VisionPortal and AprilTagProcessor lifecycle.
 * - Provides methods to retrieve detections safely across SDK versions.
 * - Returns a single "best" tag detection (closest by range) for targeting.
 *
 * FIXES:
 * - Adds tag library so pose solver knows tag size (6" / 0.1524 m),
 *   giving correct distance scaling instead of ~1200 in.
 * - Prevents stale detections from persisting once the tag leaves view.
 *   (If no new data for > 400 ms, telemetry shows "Tag Visible = false".)
 *
 * TYPICAL USAGE:
 *   VisionAprilTag vision = new VisionAprilTag();
 *   vision.init(hardwareMap, "Webcam 1");
 *   AprilTagDetection det = vision.getDetectionFor(VisionAprilTag.TAG_BLUE_GOAL);
 *   ...
 *   vision.stop();   // in OpMode.stop()
 *
 * TAG IDs (2025 DE CODE Game):
 *   Blue Goal Tag = 20
 *   Red Goal Tag  = 24
 *
 * CAMERA SETTINGS:
 * - 640×480 MJPEG stream → higher FPS + uses built-in FTC calibration.
 * - Works on older SDKs that lack some VisionPortal options.
 */

public class VisionAprilTag {
    // --- Alliance GOAL Tag IDs ---
    public static final int TAG_BLUE_GOAL = 20;
    public static final int TAG_RED_GOAL  = 24;

    // --- Constants ---
    // How long (ms) to keep using the last non-empty detection list
    // before declaring tags “not visible.”
    private static final long STALE_MS = 400;

    // --- Members ---
    private VisionPortal portal;
    private AprilTagProcessor tagProcessor;
    private long lastFreshListMs = 0; // timestamp of last valid detection list

    /**
     * Initialize the webcam + AprilTag processor.
     * @param hw          HardwareMap from the active OpMode
     * @param webcamName  Name of the webcam in the Robot Configuration (e.g. "Webcam 1")
     */
    public void init(HardwareMap hw, String webcamName) {

        // -----------------------------------------------------------
        // BUILD APRILTAG PROCESSOR
        // -----------------------------------------------------------
        AprilTagProcessor.Builder tp = new AprilTagProcessor.Builder()
                .setDrawAxes(true)             // draw small XYZ axes on detected tags
                .setDrawCubeProjection(true)   // draw cube visualization
                .setDrawTagID(true);           // draw tag ID number on frame

        // -----------------------------------------------------------
        // ASSIGN TAG LIBRARY → ensures 6" (0.1524 m) tag size for accurate range data
        // -----------------------------------------------------------
        try {
            // Try to load current game database (if SDK supports it)
            tp.setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary());
        } catch (Throwable ignoredA) {
            try {
                // Fall back to Center Stage database (also 6" tags)
                tp.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary());
            } catch (Throwable ignoredB) {
                // Build a minimal library manually if no database is available
                try {
                    double TAG_SIZE_M = 0.1524; // 6 in → 0.1524 m
                    AprilTagLibrary.Builder lb = new AprilTagLibrary.Builder();
                    lb.addTag(TAG_BLUE_GOAL, "BlueGoal", TAG_SIZE_M);
                    lb.addTag(TAG_RED_GOAL,  "RedGoal",  TAG_SIZE_M);
                    tp.setTagLibrary(lb.build());
                } catch (Throwable ignoredC) {
                    // If even this fails, pose angles will still work; range may be inaccurate.
                }
            }
        }

        tagProcessor = tp.build();

        // -----------------------------------------------------------
        // BUILD VISION PORTAL
        // -----------------------------------------------------------
        VisionPortal.Builder vp = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hw.get(WebcamName.class, webcamName))
                .setCameraResolution(new Size(640, 480)); // built-in calibration size

        // Prefer MJPEG format for higher FPS (if supported)
        try {
            vp.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        } catch (Throwable ignored) { }

        portal = vp.build();
    }

    /**
     * Return the list of AprilTag detections in a way that works on all SDKs.
     * - Prefers getFreshDetections() (newer SDKs) → latest frame.
     * - Falls back to getDetections() (mid SDKs) or public field "detections" (oldest SDKs).
     * - Applies a ~400 ms timeout so stale data is discarded when no tags are seen.
     */
    @SuppressWarnings("unchecked")
    public List<AprilTagDetection> getDetectionsCompat() {
        if (tagProcessor == null) return Collections.emptyList();

        List<AprilTagDetection> list = Collections.emptyList();
        boolean gotSomething = false;

        // Try newest API
        try {
            list = tagProcessor.getFreshDetections();
            gotSomething = true;
        } catch (Throwable ignored1) {
            try {
                list = tagProcessor.getDetections();
                gotSomething = true;
            } catch (Throwable ignored2) {
                try {
                    java.lang.reflect.Field f = tagProcessor.getClass().getField("detections");
                    Object val = f.get(tagProcessor);
                    if (val instanceof List) {
                        list = (List<AprilTagDetection>) val;
                        gotSomething = true;
                    }
                } catch (Throwable ignored3) { }
            }
        }

        long now = System.currentTimeMillis();

        if (gotSomething && list != null && !list.isEmpty()) {
            // Valid detections received → update fresh timestamp
            lastFreshListMs = now;
            return list;
        }

        // If no new data and too old, return empty to clear telemetry
        if (now - lastFreshListMs > STALE_MS) {
            return Collections.emptyList();
        }

        // Within timeout window → still return empty so display drops old values
        return Collections.emptyList();
    }

    /**
     * Return the "best" detection for a given tag ID (closest by range), or null if none visible.
     * Filters out nonsensical ranges (e.g., > 3000 in or negative).
     */
    public AprilTagDetection getDetectionFor(int desiredId) {
        AprilTagDetection best = null;
        for (AprilTagDetection d : getDetectionsCompat()) {
            if (d.id == desiredId) {
                if (d.ftcPose != null && d.ftcPose.range > 0 && d.ftcPose.range < 3000) {
                    if (best == null || d.ftcPose.range < best.ftcPose.range) best = d;
                }
            }
        }
        return best;
    }

    /**
     * Stop the Vision Portal and release camera resources.
     * Safe to call multiple times or from OpMode.stop().
     */
    public void stop() {
        try {
            if (portal != null) portal.close();
        } catch (Throwable ignored) { }
        portal = null;
        tagProcessor = null;
    }
}
