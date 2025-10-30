package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

// NEW: shared latch for obelisk signal
import org.firstinspires.ftc.teamcode.utils.ObeliskSignal;

/*
 * FILE: VisionAprilTag.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vision/
 *
 * PURPOSE
 *   - Provide a cross-SDK AprilTag helper that initializes VisionPortal +
 *     AprilTagProcessor, exposes compatibility getters, and returns the closest
 *     detection for a requested tag.
 *   - Maintain a configurable range scale to correct distance output and latch
 *     DECODE Obelisk tags (IDs 21/22/23) during the init loop.
 *
 * ALLIANCE TAG IDS (DECODE field)
 *   - BLUE GOAL: 20
 *   - RED  GOAL: 24
 *
 * OBELISK TAG IDS (DECODE field)
 *   - 21 → GPP, 22 → PGP, 23 → PPG (latched via ObeliskSignal)
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Vision & range calibration)
 *   - setRangeScale(scale)
 *       • After a tape-measure calibration compute scale = true_distance_m /
 *         measured_distance_m.
 *       • TeleOpAllianceBase currently overrides to 0.03 for the shared instance;
 *         update whenever camera height or lens changes.
 *
 * METHODS
 *   - init(hw, "Webcam 1")
 *       • Build VisionPortal + AprilTagProcessor with compatible defaults and
 *         enable the Driver Station live view stream.
 *   - getDetectionsCompat()
 *       • Return detections using the best API available for the installed SDK.
 *   - getDetectionFor(int tagId)
 *       • Return the closest visible detection for the requested tag.
 *   - setRangeScale(double s) / getScaledRange(det)
 *       • Adjust and read calibrated distance values.
 *   - observeObelisk(...) / setObeliskAutoLatchEnabled(boolean)
 *       • Track Obelisk tags during the init loop or while moving in Auto.
 *   - stop()
 *       • Close the VisionPortal when TeleOp ends.
 *
 * NOTES
 *   - We use 640x480 with MJPEG when available (built-in calibration + better FPS).
 *   - No AprilTagLibrary.addTag() calls so this compiles on older SDKs.
 *   - Bearing/pose values come from AprilTagDetection.ftcPose (meters + degrees).
 */
public class VisionAprilTag {

    // CHANGES (2025-10-30): Enabled Driver Station live stream via VisionPortal builder and resume call.

    // === CONSTANTS ===
    public static final int TAG_BLUE_GOAL = 20; // Blue alliance GOAL tag
    public static final int TAG_RED_GOAL  = 24; // Red alliance GOAL tag

    // NEW: Obelisk IDs
    public static final int TAG_OBELISK_GPP = 21;
    public static final int TAG_OBELISK_PGP = 22;
    public static final int TAG_OBELISK_PPG = 23;

    // === INTERNAL OBJECTS ===
    private VisionPortal portal;
    private AprilTagProcessor tagProcessor;

    // === RANGE SCALING ===
    // Multiply raw ftcPose.range (meters) by this factor to correct distance.
    // Set via setRangeScale() after quick tape-measure calibration.
    private double rangeScale = 1.0;

    // === OBELISK BACKGROUND POLLER (optional, for Auto) ===
    private volatile boolean obeliskAutoLatch = false;
    private Thread obeliskThread = null;

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
                .setCameraResolution(new Size(640, 480))   // Built-in calibration res
                .enableLiveView(true);                     // Stream to Driver Station DS preview

        // Prefer MJPEG for higher FPS (older SDKs may not support; ignore if so)
        try { builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG); } catch (Exception ignored) {}

        portal = builder.build();

        try { portal.resumeStreaming(); } catch (Throwable ignored) {}
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
    //  METHODS: observeObelisk
    //  PURPOSE:
    //     - Look through detections for obelisk tags (21,22,23).
    //     - If found, update shared memory so Auto/TeleOp can read it.
    //     - Call this once per loop in TeleOp and during Auto init.
    // =============================================================
    public void observeObelisk() {
        List<AprilTagDetection> dets = getDetectionsCompat();
        observeObelisk(dets);
    }

    public void observeObelisk(List<AprilTagDetection> dets) {
        if (dets == null) return;
        for (AprilTagDetection d : dets) {
            if (d == null) continue;
            int id = d.id;
            if (id == TAG_OBELISK_GPP || id == TAG_OBELISK_PGP || id == TAG_OBELISK_PPG) {
                ObeliskSignal.updateFromTagId(id);
                return; // latch first seen
            }
        }
    }

    // =============================================================
    //  METHOD: setObeliskAutoLatchEnabled  (NEW)
    //  PURPOSE:
    //     - Run a tiny background poller during Auto so tags seen while the
    //       robot is executing motion still latch immediately.
    // =============================================================
    public void setObeliskAutoLatchEnabled(boolean enable) {
        if (enable == obeliskAutoLatch) return;
        obeliskAutoLatch = enable;
        if (enable) startObeliskThread(); else stopObeliskThread();
    }

    private void startObeliskThread() {
        if (obeliskThread != null && obeliskThread.isAlive()) return;
        obeliskThread = new Thread(() -> {
            try {
                while (obeliskAutoLatch) {
                    try { observeObelisk(); } catch (Throwable ignored) {}
                    try { Thread.sleep(70); } catch (InterruptedException ie) { break; }
                }
            } finally { /* no-op */ }
        }, "VA-ObeliskLatch");
        obeliskThread.setDaemon(true);
        obeliskThread.start();
    }

    private void stopObeliskThread() {
        if (obeliskThread != null) {
            try {
                obeliskAutoLatch = false;
                obeliskThread.join(150);
            } catch (InterruptedException ignored) {
            } finally {
                obeliskThread = null;
            }
        }
    }

    // =============================================================
    //  METHOD: stop
    //  PURPOSE:
    //     - Safely close the VisionPortal when OpMode ends.
    // =============================================================
    public void stop() {
        // ensure background poller is shut down
        setObeliskAutoLatchEnabled(false);

        if (portal != null) {
            try { portal.stopStreaming(); } catch (Throwable ignored) {}
            portal.close();
        }
    }
}
