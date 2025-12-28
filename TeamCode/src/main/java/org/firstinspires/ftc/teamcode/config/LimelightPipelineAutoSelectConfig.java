package org.firstinspires.ftc.teamcode.config;

/*
 * FILE: LimelightPipelineAutoSelectConfig.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/
 *
 * PURPOSE
 *   - Centralize Limelight pipeline auto-selection tunables so TeleOp and Auto
 *     share the same INIT-time evaluation behavior without code edits.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Vision & range calibration)
 *   - ENABLE_LIMELIGHT_PIPELINE_AUTOSELECT
 *       • Master switch for INIT pipeline auto-selection in TeleOp + Auto.
 *   - LIMELIGHT_PIPELINE_PROFILES
 *       • Ordered list of pipelineIndex:description entries to evaluate.
 *         Format: "0:Meet Gym;1:Practice Field;2:Practice Field (Dim)".
 *   - PIPELINE_SETTLE_MS
 *       • Time to wait after switching pipelines before sampling frames (ms).
 *   - PIPELINE_SAMPLE_COUNT
 *       • Number of frames to sample per pipeline before scoring.
 *   - PIPELINE_SAMPLE_INTERVAL_MS
 *       • Delay between samples while scoring a pipeline (ms).
 *   - PIPELINE_MAX_SELECTION_MS
 *       • Max time allowed for selection before timing out (ms).
 *
 * CHANGES (2025-12-28): Added Limelight pipeline auto-selection tunables for
 *                       INIT-time profile evaluation and telemetry.
 */
public final class LimelightPipelineAutoSelectConfig {
    private LimelightPipelineAutoSelectConfig() {}

    public static boolean ENABLE_LIMELIGHT_PIPELINE_AUTOSELECT = true; // Enable INIT auto-selection (TeleOp + Auto)
    public static String LIMELIGHT_PIPELINE_PROFILES = "0:Meet Gym;1:Practice Field;2:Practice Field (Dim)"; // Ordered pipeline profiles to test
    public static long PIPELINE_SETTLE_MS = 250L; // Settling time after pipeline switch (ms)
    public static int PIPELINE_SAMPLE_COUNT = 6; // Frames sampled per pipeline
    public static long PIPELINE_SAMPLE_INTERVAL_MS = 60L; // Delay between samples (ms)
    public static long PIPELINE_MAX_SELECTION_MS = 2000L; // Max total selection time (ms)
}
