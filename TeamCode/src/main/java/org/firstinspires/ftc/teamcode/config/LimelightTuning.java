package org.firstinspires.ftc.teamcode.config;

/** Limelight device/pipeline knobs retained from the previous robot's tuning structure. */
public final class LimelightTuning {
    private LimelightTuning() { }
    public static int POLL_HZ = 30;
    public static int PIPELINE_INDEX = 0;
    public static boolean AUTO_SELECT = false;
    // Historical slots: verify that these pipelines exist on the selected Limelight.
    public static int[] PIPELINES = {0, 1, 2};
    public static long SETTLE_MS = 250;
    public static int SAMPLE_COUNT = 6;
    public static long SAMPLE_INTERVAL_MS = 60;
    public static long MAX_SELECTION_MS = 5000;
    public static int MIN_TARGET_HITS = 5;
    public static int FALLBACK_INDEX = 0;
    // Empty means any fiducial may qualify a pipeline. No inherited goal/obelisk IDs.
    public static int[] QUALIFYING_TAG_IDS = {};
}
