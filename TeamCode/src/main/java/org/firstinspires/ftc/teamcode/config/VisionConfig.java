package org.firstinspires.ftc.teamcode.config;

import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

/** Device selection and explicit new-season target choice, separate from camera tuning. */
public final class VisionConfig {
    private VisionConfig() { }
    public enum Source { WEBCAM, LIMELIGHT }
    public static Source SOURCE = Source.WEBCAM;
    public static String LIMELIGHT_NAME = "limelight";
    // No DECODE IDs: an unset ID prevents aim assist from choosing a game target.
    public static int TARGET_TAG_ID = -1;
    public static boolean ENABLE_AIM_ASSIST = false;
    public static long MAX_TARGET_AGE_MS = 120;
    public static double AIM_BEARING_OFFSET_DEG = 0;

    public static AprilTagLibrary createTagLibrary() {
        // Add new-season tag metadata here, or return the published season library when available.
        return new AprilTagLibrary.Builder().build();
    }
}
