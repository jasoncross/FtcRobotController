package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

// NEW: shared latch for obelisk signal
import org.firstinspires.ftc.teamcode.utils.ObeliskSignal;
import org.firstinspires.ftc.teamcode.config.VisionTuning;

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
 *   - CHANGES (2025-11-03): Apply tunable 1280x720@20 FPS camera settings with
 *     manual exposure/gain, optional white balance lock, and expose runtime
 *     performance telemetry (FPS, latency, stream status) for TeleOp display.
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

    // === PERFORMANCE METRICS ===
    private double lastKnownFps = Double.NaN;
    private double lastFrameLatencyMs = Double.NaN;
    private long lastPerfSampleMs = 0L;

    // === CAMERA CONTROL APPLICATION ===
    private volatile boolean controlsThreadStarted = false;
    private volatile boolean controlsApplied = false;
    private Thread controlsThread = null;
    private String controlWarningOnce = null;

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
        try { tagProcessor.setDecimation(VisionTuning.APRILTAG_DECIMATION); } catch (Throwable ignored) {}

        // Build VisionPortal
        VisionPortal.Builder builder = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hw.get(WebcamName.class, webcamName))
                .setCameraResolution(new Size(
                        VisionTuning.VISION_RES_WIDTH,
                        VisionTuning.VISION_RES_HEIGHT))
                .enableLiveView(true);                     // Stream to Driver Station DS preview

        // Prefer YUY2 for 720p stability; fall back to MJPEG if unavailable.
        boolean streamFormatSet = false;
        try {
            builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
            streamFormatSet = true;
        } catch (Throwable ignored) { /* fall back below */ }
        if (!streamFormatSet) {
            try { builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG); } catch (Throwable ignored) {}
        }

        portal = builder.build();

        applyTargetFpsLimit();
        scheduleCameraControlApply();

        try { portal.resumeStreaming(); } catch (Throwable ignored) {}
    }

    private void scheduleCameraControlApply() {
        if (portal == null) return;
        if (controlsThreadStarted) return;
        controlsThreadStarted = true;
        controlsThread = new Thread(() -> {
            try {
                long start = System.currentTimeMillis();
                boolean streaming = false;
                while (!Thread.currentThread().isInterrupted()) {
                    try {
                        VisionPortal.CameraState state = null;
                        try { state = portal.getCameraState(); } catch (Throwable ignored) {}
                        if (state == VisionPortal.CameraState.STREAMING) { streaming = true; break; }
                        if ((System.currentTimeMillis() - start) > 2500) break;
                        Thread.sleep(40);
                    } catch (InterruptedException ie) {
                        Thread.currentThread().interrupt();
                        return;
                    }
                }
                if (streaming) {
                    applyCameraControls();
                    controlsApplied = true;
                }
            } finally {
                controlsThread = null;
            }
        }, "VA-CamCtrl");
        controlsThread.setDaemon(true);
        controlsThread.start();
    }

    private void applyCameraControls() {
        if (portal == null) return;
        StringBuilder warn = new StringBuilder();
        if (!applyExposure()) warn.append(" Exposure");
        if (!applyGain()) warn.append(" Gain");
        if (!applyWhiteBalance()) warn.append(" WhiteBalance");
        if (warn.length() > 0 && controlWarningOnce == null) {
            controlWarningOnce = String.format(Locale.US,
                    "Vision control unsupported: %s", warn.toString().trim());
        }
    }

    private boolean applyExposure() {
        try {
            ExposureControl exposureControl = portal.getCameraControl(ExposureControl.class);
            if (exposureControl == null) return false;
            try {
                if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                    exposureControl.setMode(ExposureControl.Mode.Manual);
                }
            } catch (Throwable ignored) {}
            exposureControl.setExposure(VisionTuning.EXPOSURE_MS, TimeUnit.MILLISECONDS);
            return true;
        } catch (Throwable ignored) {
            return false;
        }
    }

    private boolean applyGain() {
        try {
            GainControl gainControl = portal.getCameraControl(GainControl.class);
            if (gainControl == null) return false;
            gainControl.setGain(VisionTuning.GAIN);
            return true;
        } catch (Throwable ignored) {
            return false;
        }
    }

    @SuppressWarnings("rawtypes")
    private boolean applyWhiteBalance() {
        try {
            Class<?> wbClass = Class.forName("org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl");
            Object wbControl = portal.getCameraControl((Class) wbClass);
            if (wbControl == null) return false;

            boolean desiredLock = VisionTuning.WHITE_BALANCE_LOCK_ENABLED;
            boolean handled = invokeWhiteBalanceLock(wbClass, wbControl, desiredLock);
            if (!handled) handled = invokeWhiteBalanceMode(wbClass, wbControl, desiredLock);
            return handled;
        } catch (Throwable ignored) {
            return false;
        }
    }

    private boolean invokeWhiteBalanceLock(Class<?> wbClass, Object control, boolean desiredLock) {
        try {
            for (Method m : wbClass.getMethods()) {
                if (!m.getName().equals("setWhiteBalanceLocked")) continue;
                Class<?>[] params = m.getParameterTypes();
                if (params.length != 1) continue;
                if (params[0] == boolean.class || params[0] == Boolean.class) {
                    m.invoke(control, desiredLock);
                    return true;
                }
            }
        } catch (Throwable ignored) {}
        return false;
    }

    private boolean invokeWhiteBalanceMode(Class<?> wbClass, Object control, boolean desiredLock) {
        try {
            for (Method m : wbClass.getMethods()) {
                if (!m.getName().equals("setMode")) continue;
                Class<?>[] params = m.getParameterTypes();
                if (params.length != 1) continue;
                Class<?> modeClass = params[0];
                if (!modeClass.isEnum()) continue;
                Object[] constants = modeClass.getEnumConstants();
                Object manual = null, auto = null;
                for (Object constant : constants) {
                    String name = constant.toString().toUpperCase(Locale.US);
                    if (manual == null && (name.contains("MANUAL") || name.contains("LOCK"))) manual = constant;
                    if (auto == null && name.contains("AUTO")) auto = constant;
                }
                if (desiredLock && manual != null) {
                    m.invoke(control, manual);
                    return true;
                }
                if (!desiredLock && auto != null) {
                    m.invoke(control, auto);
                    return true;
                }
            }
        } catch (Throwable ignored) {}
        return false;
    }

    private void applyTargetFpsLimit() {
        if (portal == null || tagProcessor == null) return;
        double fps = VisionTuning.VISION_TARGET_FPS;
        if (fps <= 0) return;
        try {
            for (Method m : portal.getClass().getMethods()) {
                if (!m.getName().equals("setProcessorFpsLimit")) continue;
                Class<?>[] params = m.getParameterTypes();
                if (params.length != 2) continue;
                if (!params[0].isInstance(tagProcessor) && !params[0].isAssignableFrom(tagProcessor.getClass())) continue;
                if (params[1] == double.class || params[1] == Double.TYPE) {
                    m.invoke(portal, tagProcessor, fps);
                    return;
                } else if (params[1] == float.class || params[1] == Float.TYPE) {
                    m.invoke(portal, tagProcessor, (float) fps);
                    return;
                } else if (params[1] == int.class || params[1] == Integer.TYPE) {
                    m.invoke(portal, tagProcessor, (int)Math.round(fps));
                    return;
                }
            }
        } catch (Throwable ignored) {
            // leave unlimited if SDK does not support FPS limits
        }
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

        samplePerformanceMetrics();

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
        updateLatencyFromDetection(best);
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

    public void samplePerformanceMetrics() {
        if (portal == null) return;
        long now = System.currentTimeMillis();
        if ((now - lastPerfSampleMs) < 40) return; // ~25 Hz max sampling to reduce reflection churn
        lastPerfSampleMs = now;

        Double fps = invokeDoubleNoArgs(portal, "getFps", "getFramesPerSecond");
        if (fps != null) lastKnownFps = fps;

        Double latency = invokeDoubleNoArgs(portal,
                "getFrameLatencyMillis",
                "getFrameLatencyMs",
                "getCurrentLatencyMs",
                "getLatestFrameLatencyMs");
        if (latency != null) lastFrameLatencyMs = latency;
    }

    public Double getLastKnownFps() {
        return Double.isNaN(lastKnownFps) ? null : lastKnownFps;
    }

    public Double getLastFrameLatencyMs() {
        return Double.isNaN(lastFrameLatencyMs) ? null : lastFrameLatencyMs;
    }

    public boolean isStreamActive() {
        if (portal == null) return false;
        try {
            return portal.getCameraState() == VisionPortal.CameraState.STREAMING;
        } catch (Throwable ignored) {
            return false;
        }
    }

    public boolean wereControlsApplied() {
        return controlsApplied;
    }

    public String consumeControlWarning() {
        String msg = controlWarningOnce;
        controlWarningOnce = null;
        return msg;
    }

    private Double invokeDoubleNoArgs(Object target, String... methodNames) {
        for (String name : methodNames) {
            try {
                Method m = target.getClass().getMethod(name);
                Object val = m.invoke(target);
                if (val instanceof Number) {
                    return ((Number) val).doubleValue();
                }
            } catch (Throwable ignored) {
                // try next method name
            }
        }
        return null;
    }

    private void updateLatencyFromDetection(AprilTagDetection det) {
        if (det == null) return;
        try {
            Field metaField;
            try {
                metaField = det.getClass().getField("metadata");
            } catch (NoSuchFieldException nsf) {
                metaField = det.getClass().getDeclaredField("metadata");
                metaField.setAccessible(true);
            }
            Object meta = metaField.get(det);
            Double latency = extractLatencyFromMetadata(meta);
            if (latency != null) {
                lastFrameLatencyMs = latency;
            }
        } catch (Throwable ignored) {
            // ignore missing metadata
        }
    }

    private Double extractLatencyFromMetadata(Object metadata) {
        if (metadata == null) return null;
        long now = System.nanoTime();
        String[] candidateFields = new String[] {
                "frameAcquisitionNanoTime",
                "frameAcquisitionTimeNanos",
                "frameTimestampNanos",
                "captureTimeNanos"
        };
        for (String fieldName : candidateFields) {
            try {
                Field f;
                try {
                    f = metadata.getClass().getField(fieldName);
                } catch (NoSuchFieldException nsf) {
                    f = metadata.getClass().getDeclaredField(fieldName);
                    f.setAccessible(true);
                }
                Object val = f.get(metadata);
                if (val instanceof Number) {
                    long nanos = ((Number) val).longValue();
                    if (nanos > 0 && now > nanos) {
                        return (now - nanos) / 1_000_000.0;
                    }
                }
            } catch (Throwable ignored) {
                // try next field name
            }
        }
        return null;
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

        if (controlsThread != null) {
            try { controlsThread.interrupt(); } catch (Throwable ignored) {}
            controlsThread = null;
        }
        controlsThreadStarted = false;
        controlsApplied = false;

        if (portal != null) {
            try { portal.stopStreaming(); } catch (Throwable ignored) {}
            portal.close();
        }
    }
}
