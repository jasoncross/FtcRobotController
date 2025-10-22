package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor.*;

// Camera controls (optional but recommended)
import android.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;

import java.util.Collections;
import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * FILE: VisionAprilTag.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vision/
 *
 * PURPOSE:
 * - Owns VisionPortal + AprilTagProcessor for AprilTag detection and pose.
 * - Provides helpers to get the "best" (closest) detection for the goal tag id,
 *   plus raw detection access for telemetry/debug.
 *
 * ALLIANCE GOAL TAGS:
 * - BLUE: 20
 * - RED : 24
 *
 * CAMERA / STREAM:
 * - Default to MJPEG @ 640x480 for higher FPS and SDK’s built-in calibration.
 * - If you change to 1280x720 (or other), either accept pose warnings or set lens intrinsics.
 *
 * CALIBRATION:
 * - For best distance accuracy:
 *   1) Use 640x480 (built-in) OR call configureIntrinsics(...) with your lens params.
 *   2) If small residual error remains, tune distanceScale below (e.g., 0.95–1.05).
 *
 * METHODS:
 * - init(hw, webcamName):            create VisionPortal + AprilTagProcessor (ready to use).
 * - setManualExposure(ms, gain):     optional — force exposure/gain (prevents motion blur).
 * - configureIntrinsics(...):        optional — supply lens intrinsics for your camera/res.
 * - setDistanceScale(s):             optional — scalar applied to ftcPose.range.
 * - getDetectionFor(id):             return closest detection for a given id (or null).
 * - getAllDetections():              return all current detections (never null).
 * - stop():                          close the portal on OpMode stop().
 */

public class VisionAprilTag {

    // ---------------- Constants ----------------
    public static final int TAG_BLUE_GOAL = 20; // Blue alliance GOAL tag
    public static final int TAG_RED_GOAL  = 24; // Red alliance GOAL tag

    // ---------------- Members ----------------
    private VisionPortal portal;
    private AprilTagProcessor tagProcessor;

    // Optional calibration & tuning
    private double distanceScale = 1.00; // quick scalar to nudge range (meters). Tune on-field: 0.95–1.05 typical.

    /**
     * Initialize the camera + AprilTag processor.
     * @param hw          HardwareMap
     * @param webcamName  Must match Robot Configuration (e.g., "Webcam 1")
     */
    public void init(HardwareMap hw, String webcamName) {
        // ==============================================================
        // APRILTAG PROCESSOR
        // - Using TAG_36h11 family (FTC standard).
        // - Output pose in METERS / DEGREES (matches our telemetry/logic).
        // - Decimation reduces input image scale for speed (2–3 typical).
        // ==============================================================
        AprilTagProcessor.Builder tp = new AprilTagProcessor.Builder()
                .setTagFamily(TagFamily.TAG_36h11)
                .setOutputUnits(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.METER,
                                org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDecimation(2); // try 2 or 3; 1 = full-res (slower, more range)

        tagProcessor = tp.build();

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

    // ==============================================================
    // OPTIONAL CAMERA CONTROLS (EXPOSURE/GAIN)
    // - Calling this after init() can dramatically stabilize AprilTag reads.
    // - Suggested starting point: exposure 6–12 ms, gain 0–8 (varies by cam/lighting).
    // ==============================================================
    public void setManualExposure(int exposureMs, int gain) {
        if (portal == null) return;
        try {
            ExposureControl ec = portal.getCameraControl(ExposureControl.class);
            if (ec != null) {
                ec.setMode(ExposureControl.Mode.Manual);
                ec.setExposure(exposureMs, TimeUnit.MILLISECONDS);
            }
            GainControl gc = portal.getCameraControl(GainControl.class);
            if (gc != null) {
                // Clamp gain to camera’s supported range
                Range<Integer> r = gc.getGainRange();
                int clamped = Math.max(r.getLower(), Math.min(r.getUpper(), gain));
                gc.setGain(clamped);
            }
        } catch (Throwable t) {
            // Ignore if camera doesn’t support these controls
        }
    }

    // ==============================================================
    // OPTIONAL LENS INTRINSICS
    // - If you must run 1280x720 or a camera without built-in 640x480 calibration,
    //   you can provide fx, fy, cx, cy here. Values are in pixels for the chosen resolution.
    // ==============================================================
    public void configureIntrinsics(double fx, double fy, double cx, double cy) {
        if (tagProcessor != null) {
            tagProcessor.setLensIntrinsics(fx, fy, cx, cy);
        }
    }

    // Quick scalar to nudge reported range (meters) to match a tape-measured truth.
    public void setDistanceScale(double s) { this.distanceScale = s; }

    /**
     * Return the closest detection for the desired tag id, or null if none are visible.
     * "Closest" = smallest ftcPose.range (meters).
     * NOTE: Applies distanceScale to returned detection's range for telemetry/use.
     */
    public AprilTagDetection getDetectionFor(int desiredId) {
        if (tagProcessor == null) return null;

        AprilTagDetection best = null;
        for (AprilTagDetection d : tagProcessor.getDetections()) {
            if (d.id == desiredId) {
                if (best == null || d.ftcPose.range < best.ftcPose.range) {
                    best = d;
                }
            }
        }
        if (best != null && distanceScale != 1.0) {
            // Adjust range in-place for telemetry/logic (bearing/yaw unaffected)
            best.ftcPose.range *= distanceScale;
        }
        return best;
    }

    /** Return a snapshot list of all current detections (never null). */
    public List<AprilTagDetection> getAllDetections() {
        if (tagProcessor == null) return Collections.emptyList();
        return tagProcessor.getDetections();
    }

    /** Close the VisionPortal and free camera resources. */
    public void stop() {
        if (portal != null) {
            portal.close();
            portal = null;
        }
    }
}
