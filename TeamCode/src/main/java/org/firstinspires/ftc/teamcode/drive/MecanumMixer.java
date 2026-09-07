package org.firstinspires.ftc.teamcode.drive;

/** Pure wheel-power calculation, independent of hardware. */
public final class MecanumMixer {
    private MecanumMixer() { }

    /** Returns front-left, front-right, back-left, back-right, normalized together. */
    public static double[] mix(double forward, double strafeRight, double clockwise, double limit) {
        if (!Double.isFinite(forward) || !Double.isFinite(strafeRight)
                || !Double.isFinite(clockwise) || !Double.isFinite(limit)) {
            return new double[4];
        }
        double[] powers = {forward + strafeRight + clockwise,
                forward - strafeRight - clockwise,
                forward - strafeRight + clockwise,
                forward + strafeRight - clockwise};
        double denominator = 1.0;
        for (double power : powers) denominator = Math.max(denominator, Math.abs(power));
        double scale = Math.max(0, Math.min(1, limit)) / denominator;
        for (int i = 0; i < powers.length; i++) powers[i] *= scale;
        return powers;
    }
}
