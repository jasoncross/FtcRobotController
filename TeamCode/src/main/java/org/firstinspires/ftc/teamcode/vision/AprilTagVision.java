package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.config.VisionConfig;
import org.firstinspires.ftc.teamcode.config.VisionTuning;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/** Webcam AprilTag detection with an explicit library; no inherited game targets or poses. */
public final class AprilTagVision implements VisionTargetProvider {
    private final AprilTagProcessor processor;
    private final VisionPortal portal;
    private final VisionTuning.Profile profile;
    private boolean controlsAttempted;
    private String controlStatus = "SDK camera controls";

    /** Detect IDs and image centers only until measured tag metadata is supplied. */
    public AprilTagVision(HardwareMap hardwareMap) {
        this(hardwareMap, VisionConfig.createTagLibrary());
    }

    public AprilTagVision(HardwareMap hardwareMap, AprilTagLibrary tagLibrary) {
        profile = VisionTuning.selectedProfile();
        AprilTagProcessor.Builder builder = new AprilTagProcessor.Builder()
                .setTagLibrary(tagLibrary).setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES);
        if (VisionTuning.USE_CALIBRATED_INTRINSICS) {
            if (profile.fx <= 0 || profile.fy <= 0) throw new IllegalArgumentException("Selected profile has no intrinsics");
            builder.setLensIntrinsics(profile.fx, profile.fy, profile.cx, profile.cy);
        }
        processor = builder.build();
        processor.setDecimation(profile.decimation);
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, RobotConfig.WEBCAM_NAME))
                .setCameraResolution(new Size(profile.width, profile.height))
                .enableLiveView(VisionTuning.LIVE_VIEW_ENABLED)
                .addProcessor(processor)
                .build();
    }

    public List<AprilTagDetection> getDetections() {
        List<AprilTagDetection> filtered = new ArrayList<>();
        long now = System.nanoTime();
        for (AprilTagDetection tag : processor.getDetections()) {
            long age = now - tag.frameAcquisitionNanoTime;
            if (age >= 0 && age / 1_000_000.0 <= VisionConfig.MAX_TARGET_AGE_MS
                    && tag.decisionMargin >= profile.minDecisionMargin) filtered.add(tag);
        }
        return filtered;
    }

    @Override public void update() {
        if (!VisionTuning.USE_MANUAL_CONTROLS || controlsAttempted
                || portal.getCameraState() != VisionPortal.CameraState.STREAMING) return;
        controlsAttempted = true;
        if (profile.exposureMs <= 0) { controlStatus = "No manual settings in selected profile"; return; }
        try {
            ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
            boolean exposureOk = exposure != null && exposure.isModeSupported(ExposureControl.Mode.Manual)
                    && exposure.setMode(ExposureControl.Mode.Manual);
            if (exposureOk) exposureOk = exposure.setExposure(Math.max(exposure.getMinExposure(TimeUnit.MILLISECONDS),
                    Math.min(exposure.getMaxExposure(TimeUnit.MILLISECONDS), profile.exposureMs)), TimeUnit.MILLISECONDS);
            GainControl gain = portal.getCameraControl(GainControl.class);
            boolean gainOk = gain != null && gain.setGain(Math.max(gain.getMinGain(), Math.min(gain.getMaxGain(), profile.gain)));
            boolean wbOk = true;
            if (profile.lockWhiteBalance) {
                WhiteBalanceControl white = portal.getCameraControl(WhiteBalanceControl.class);
                int temperature = white == null ? 0 : white.getWhiteBalanceTemperature();
                wbOk = white != null && white.setMode(WhiteBalanceControl.Mode.MANUAL)
                        && white.setWhiteBalanceTemperature(temperature);
            }
            controlStatus = "Manual exposure=" + exposureOk + " gain=" + gainOk + " WB=" + wbOk;
        } catch (UnsupportedOperationException | IllegalArgumentException e) {
            controlStatus = "Camera control unsupported: " + e.getMessage();
        }
    }

    @Override public List<TargetObservation> getTargets() {
        List<TargetObservation> targets = new ArrayList<>();
        for (AprilTagDetection tag : getDetections()) {
            // SDK bearing is positive left; shared drive/vision bearing is positive right.
            double bearing = tag.ftcPose == null ? Double.NaN : -tag.ftcPose.bearing;
            double range = tag.ftcPose == null ? Double.NaN : tag.ftcPose.range;
            targets.add(new TargetObservation(tag.id, bearing, range, tag.frameAcquisitionNanoTime));
        }
        return targets;
    }

    @Override public String getStatus() {
        return "Webcam " + getCameraState() + " " + VisionTuning.PRESET + ": " + controlStatus;
    }

    public VisionPortal.CameraState getCameraState() {
        return portal.getCameraState();
    }

    @Override
    public void close() {
        portal.close();
    }
}
