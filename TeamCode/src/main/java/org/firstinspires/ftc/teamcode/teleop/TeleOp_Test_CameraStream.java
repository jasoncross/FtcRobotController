package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.TeleOpDriverDefaults;
import org.firstinspires.ftc.teamcode.config.VisionTuning;
import org.firstinspires.ftc.teamcode.drive.Drivebase;
import org.firstinspires.ftc.teamcode.vision.VisionAprilTag;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;
import java.util.Locale;

/**
 * FILE: TeleOp_Test_CameraStream.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleop/
 *
 * PURPOSE
 *   - Provide a minimal TeleOp focused on pit-side camera validation—keeping
 *     only drivetrain control, AprilTag detection, and live streaming enabled.
 *   - Allow rapid toggling between the tuned 480p performance stream and the
 *     720p sighting stream without the rest of the competitive TeleOp stack.
 *
 * METHODS
 *   - init()  → Spin up drivetrain + vision and enable the live stream.
 *   - loop()  → Drive, surface tag telemetry, and service resolution swaps.
 *   - stop()  → Halt the drivetrain and tear down the VisionPortal cleanly.
 *
 * CHANGES (2025-11-11): Initial camera-stream test mode supporting drivetrain,
 *                       AprilTag telemetry, and 480p/720p swaps on the D-pad.
 * CHANGES (2025-12-03): Added a 2.5s vision health sampler (Gamepad 1 A) with
 *                       decision-margin/brightness summaries and suggestions,
 *                       plus an option to preview normalized frames while
 *                       testing lighting.
 */
@TeleOp(name = "X - Test - Camera Stream", group = "TeleOp")
public final class TeleOp_Test_CameraStream extends OpMode {

    private static final double M_TO_IN = 39.37007874015748;

    private Drivebase drive;
    private VisionAprilTag vision;

    private volatile boolean profileSwapInProgress = false;
    private volatile String profileSwapError = null;
    private VisionTuning.Mode pendingMode = null;
    private Thread profileSwapThread;

    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastAPressed = false;
    private boolean samplingActive = false;
    private long samplingStartMs = 0L;
    private int samplingTotalSamples = 0;
    private int samplingGoodSamples = 0;
    private double samplingMarginSum = 0.0;
    private int samplingMarginCount = 0;
    private double samplingMarginMin = Double.POSITIVE_INFINITY;
    private double samplingMarginMax = Double.NEGATIVE_INFINITY;
    private double samplingBrightnessSum = 0.0;
    private int samplingBrightnessCount = 0;
    private String lastHealthSummary = null;
    private String lastHealthSuggestion = null;

    @Override
    public void init() {
        telemetry.setMsTransmissionInterval(50);

        drive = new Drivebase(hardwareMap, telemetry);

        vision = new VisionAprilTag();
        vision.init(hardwareMap, "Webcam 1");
        vision.setRangeScale(VisionTuning.RANGE_SCALE);
        vision.setPreviewShowsNormalized(true);
        vision.toggleLiveView(true); // keep the driver-station preview active for diagnostics

        telemetry.addLine("Camera stream diagnostics online – press START when ready");
        telemetry.addLine("Press A to sample vision health (~2.5s)");
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- Drivetrain: standard DECODE mapping with brake trigger scaling ---
        double brake = gamepad1.left_trigger;
        double cap = 1.0 - brake * (1.0 - TeleOpDriverDefaults.SLOWEST_SPEED);

        double driveY = cap * gamepad1.left_stick_y;
        double strafeX = cap * -gamepad1.left_stick_x;
        double twist = cap * -gamepad1.right_stick_x;

        if (drive != null) {
            drive.drive(driveY, strafeX, twist);
        }

        // --- Profile swaps (D-pad left = 480p, right = 720p) ---
        boolean dpadLeft = gamepad1.dpad_left;
        boolean dpadRight = gamepad1.dpad_right;
        boolean aPressed = gamepad1.a;

        if (dpadLeft && !lastDpadLeft) {
            requestProfileSwap(VisionTuning.Mode.P480);
        }
        if (dpadRight && !lastDpadRight) {
            requestProfileSwap(VisionTuning.Mode.P720);
        }
        lastDpadLeft = dpadLeft;
        lastDpadRight = dpadRight;
        if (aPressed && !lastAPressed && !samplingActive) {
            startHealthSampling();
        }
        lastAPressed = aPressed;

        // --- Vision telemetry ---
        if (vision != null) {
            vision.samplePerformanceMetrics();
        }

        List<AprilTagDetection> detections = (vision != null) ? vision.getDetectionsCompat() : null;
        AprilTagDetection nearest = findNearestDetection(detections);
        sampleHealthIfActive(nearest);

        telemetry.addLine("X - Test - Camera Stream");
        if (samplingActive) {
            telemetry.addLine(String.format(Locale.US, "Health sampling… %d ms left", Math.max(0, 2500 - (System.currentTimeMillis() - samplingStartMs))));
        } else {
            telemetry.addLine("Press A to sample vision health (~2.5s window)");
        }

        if (lastHealthSummary != null) {
            telemetry.addLine(lastHealthSummary);
            if (lastHealthSuggestion != null) {
                telemetry.addLine(lastHealthSuggestion);
            }
        }

        if (profileSwapInProgress) {
            String pending = (pendingMode != null) ? pendingMode.name() : "--";
            telemetry.addLine(String.format(Locale.US, "Vision: switching to %s …", pending));
        } else if (vision != null) {
            VisionTuning.Profile profile = vision.getActiveProfile();
            if (profile == null) profile = VisionTuning.forMode(VisionTuning.DEFAULT_MODE);
            String liveView = vision.isLiveViewEnabled() ? "ON" : "OFF";
            telemetry.addLine(String.format(Locale.US,
                    "Vision: Profile=%s LiveView=%s Res=%dx%d@%d Decim=%.1f ProcN=%d MinM=%.0f",
                    profile.name,
                    liveView,
                    profile.width,
                    profile.height,
                    profile.fps,
                    profile.decimation,
                    profile.processEveryN,
                    profile.minDecisionMargin));
        } else {
            telemetry.addLine("Vision: offline");
        }

        if (profileSwapError != null) {
            telemetry.addData("VisionError", profileSwapError);
        }

        if (detections != null) {
            telemetry.addData("Detections", detections.size());
        } else {
            telemetry.addData("Detections", "--");
        }

        if (nearest != null && vision != null) {
            double rangeM = vision.getScaledRange(nearest);
            double rangeIn = Double.isNaN(rangeM) ? Double.NaN : rangeM * M_TO_IN;
            telemetry.addData("Nearest",
                    "ID=%d Range=%.1f in Bearing=%.1f°",
                    nearest.id,
                    rangeIn,
                    nearest.ftcPose.bearing);
        } else {
            telemetry.addLine("Nearest: none");
        }

        if (vision != null) {
            Double fps = vision.getLastKnownFps();
            Double latency = vision.getLastFrameLatencyMs();
            String fpsStr = (fps == null) ? "---" : String.format(Locale.US, "%.1f", fps);
            String latencyStr = (latency == null) ? "---" : String.format(Locale.US, "%.0f", latency);
            telemetry.addLine(String.format(Locale.US, "Perf: FPS=%s LatMs=%s", fpsStr, latencyStr));
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        if (drive != null) {
            drive.stop();
        }
        if (vision != null) {
            vision.stop();
        }
        if (profileSwapThread != null) {
            profileSwapThread.interrupt();
            profileSwapThread = null;
        }
        profileSwapInProgress = false;
        pendingMode = null;
    }

    private void startHealthSampling() {
        samplingActive = true;
        samplingStartMs = System.currentTimeMillis();
        samplingTotalSamples = 0;
        samplingGoodSamples = 0;
        samplingMarginSum = 0.0;
        samplingMarginCount = 0;
        samplingMarginMin = Double.POSITIVE_INFINITY;
        samplingMarginMax = Double.NEGATIVE_INFINITY;
        samplingBrightnessSum = 0.0;
        samplingBrightnessCount = 0;
        lastHealthSummary = null;
        lastHealthSuggestion = null;
    }

    private void sampleHealthIfActive(AprilTagDetection nearest) {
        if (!samplingActive || vision == null) {
            return;
        }
        long now = System.currentTimeMillis();

        AprilTagDetection sampleDet = nearest;
        if (sampleDet == null) {
            sampleDet = vision.getNearestGoalTag();
        }

        samplingTotalSamples++;
        if (vision.hasGoodGoalTagForAim()) {
            samplingGoodSamples++;
        }

        if (sampleDet != null && !Double.isNaN(sampleDet.decisionMargin)) {
            samplingMarginSum += sampleDet.decisionMargin;
            samplingMarginCount++;
            samplingMarginMin = Math.min(samplingMarginMin, sampleDet.decisionMargin);
            samplingMarginMax = Math.max(samplingMarginMax, sampleDet.decisionMargin);
        }

        double brightnessMean = vision.getSmoothedBrightnessMean();
        if (!Double.isNaN(brightnessMean)) {
            samplingBrightnessSum += brightnessMean;
            samplingBrightnessCount++;
        }

        if ((now - samplingStartMs) >= 2500) {
            samplingActive = false;
            finalizeHealthReport();
        }
    }

    private void finalizeHealthReport() {
        VisionTuning.Profile profile = (vision != null) ? vision.getActiveProfile() : VisionTuning.forMode(VisionTuning.DEFAULT_MODE);
        if (profile == null) {
            profile = VisionTuning.forMode(VisionTuning.DEFAULT_MODE);
        }

        double ratio = (samplingTotalSamples > 0) ? (double) samplingGoodSamples / samplingTotalSamples : 0.0;
        double avgMargin = (samplingMarginCount > 0) ? samplingMarginSum / samplingMarginCount : Double.NaN;
        double avgBrightness = (samplingBrightnessCount > 0) ? samplingBrightnessSum / samplingBrightnessCount : Double.NaN;
        double minMargin = Double.isInfinite(samplingMarginMin) ? Double.NaN : samplingMarginMin;
        double maxMargin = Double.isInfinite(samplingMarginMax) ? Double.NaN : samplingMarginMax;

        boolean brightnessExtreme = !Double.isNaN(avgBrightness)
                && Math.abs(avgBrightness - VisionTuning.TARGET_MEAN_BRIGHTNESS) > VisionTuning.HEALTH_BRIGHTNESS_WARN_DELTA;
        boolean marginStrong = !Double.isNaN(avgMargin) && avgMargin >= profile.minDecisionMargin;
        boolean marginWeak = !Double.isNaN(avgMargin) && avgMargin >= (profile.minDecisionMargin - VisionTuning.HEALTH_MARGIN_WARN_DELTA);

        String state;
        if (samplingTotalSamples == 0 || samplingGoodSamples == 0 || brightnessExtreme || (!marginStrong && ratio < VisionTuning.HEALTH_WARN_GOOD_RATIO)) {
            state = "FAIL";
        } else if (ratio >= VisionTuning.HEALTH_PASS_GOOD_RATIO && marginStrong) {
            state = "PASS";
        } else if (ratio >= VisionTuning.HEALTH_WARN_GOOD_RATIO || marginWeak) {
            state = "WARN";
        } else {
            state = "FAIL";
        }

        lastHealthSummary = String.format(Locale.US,
                "Health %s: good=%d/%d (%.0f%%) margin avg=%s min=%s max=%s bright=%s",
                state,
                samplingGoodSamples,
                samplingTotalSamples,
                ratio * 100.0,
                Double.isNaN(avgMargin) ? "--" : String.format(Locale.US, "%.1f", avgMargin),
                Double.isNaN(minMargin) ? "--" : String.format(Locale.US, "%.1f", minMargin),
                Double.isNaN(maxMargin) ? "--" : String.format(Locale.US, "%.1f", maxMargin),
                Double.isNaN(avgBrightness) ? "--" : String.format(Locale.US, "%.0f", avgBrightness));

        lastHealthSuggestion = null;
        if (brightnessExtreme && !Double.isNaN(avgBrightness)) {
            if (avgBrightness < VisionTuning.TARGET_MEAN_BRIGHTNESS) {
                lastHealthSuggestion = "Try P480_EXPOSURE_MS += 1 (up to +2). Consider INIT_EXPOSURE_MAX_STEPS = 3.";
            } else {
                lastHealthSuggestion = "Try P480_EXPOSURE_MS -= 1. If already low, reduce P480_GAIN by ~5.";
            }
        } else if (!marginStrong) {
            lastHealthSuggestion = "Try reducing P480_MIN_DECISION_MARGIN by 2 (not below 8).";
        }
    }

    private void requestProfileSwap(VisionTuning.Mode mode) {
        if (vision == null || mode == null) {
            return;
        }
        VisionTuning.Mode active = vision.getActiveMode();
        if (!profileSwapInProgress && active == mode) {
            profileSwapError = null;
            return;
        }
        if (profileSwapInProgress && mode == pendingMode) {
            return;
        }

        pendingMode = mode;
        profileSwapError = null;
        profileSwapInProgress = true;

        profileSwapThread = new Thread(() -> {
            try {
                vision.applyProfile(mode);
                vision.toggleLiveView(true); // keep preview active after rebuilding the portal
            } catch (Throwable t) {
                String msg = t.getMessage();
                profileSwapError = (msg != null && !msg.isEmpty()) ? msg : t.getClass().getSimpleName();
            } finally {
                profileSwapInProgress = false;
            }
        }, "CameraProfileSwap");
        profileSwapThread.setDaemon(true);
        profileSwapThread.start();
    }

    private AprilTagDetection findNearestDetection(List<AprilTagDetection> detections) {
        if (detections == null || detections.isEmpty()) {
            return null;
        }
        AprilTagDetection best = null;
        double bestRange = Double.POSITIVE_INFINITY;
        for (AprilTagDetection det : detections) {
            if (det == null || det.ftcPose == null) {
                continue;
            }
            double range = det.ftcPose.range;
            if (Double.isNaN(range) || Double.isInfinite(range)) {
                continue;
            }
            if (best == null || range < bestRange) {
                best = det;
                bestRange = range;
            }
        }
        return best;
    }
}
