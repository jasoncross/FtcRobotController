package org.firstinspires.ftc.teamcode.config;

/** Reusable camera profiles. C270 numbers are retained DECODE baselines, not new calibration. */
public final class VisionTuning {
    private VisionTuning() { }
    public enum Preset { SDK_DEFAULT, C270_P480_PRACTICE, C270_P480_EVENT, C270_P720 }
    public static Preset PRESET = Preset.SDK_DEFAULT;
    public static boolean USE_MANUAL_CONTROLS = false;
    public static boolean USE_CALIBRATED_INTRINSICS = false;
    public static boolean LIVE_VIEW_ENABLED = false;

    public static final class Profile {
        public final int width, height, exposureMs, gain;
        public final float decimation;
        public final double minDecisionMargin, fx, fy, cx, cy;
        public final boolean lockWhiteBalance;
        public Profile(int width, int height, float decimation, double minDecisionMargin,
                       int exposureMs, int gain, boolean lockWhiteBalance,
                       double fx, double fy, double cx, double cy) {
            this.width = width; this.height = height; this.decimation = decimation;
            this.minDecisionMargin = minDecisionMargin; this.exposureMs = exposureMs;
            this.gain = gain; this.lockWhiteBalance = lockWhiteBalance;
            this.fx = fx; this.fy = fy; this.cx = cx; this.cy = cy;
        }
    }

    public static Profile SDK_DEFAULT = new Profile(640, 480, 2.0f, 10, 0, 0, false, 0, 0, 0, 0);
    public static Profile C270_P480_PRACTICE = new Profile(640, 480, 2.0f, 10, 6, 85, true, 690, 690, 320, 240);
    public static Profile C270_P480_EVENT = new Profile(640, 480, 2.0f, 10, 2, 50, true, 690, 690, 320, 240);
    public static Profile C270_P720 = new Profile(1280, 720, 2.2f, 24, 7, 85, true, 1380, 1035, 640, 360);

    public static Profile selectedProfile() {
        switch (PRESET) {
            case C270_P480_PRACTICE: return C270_P480_PRACTICE;
            case C270_P480_EVENT: return C270_P480_EVENT;
            case C270_P720: return C270_P720;
            default: return SDK_DEFAULT;
        }
    }
}
