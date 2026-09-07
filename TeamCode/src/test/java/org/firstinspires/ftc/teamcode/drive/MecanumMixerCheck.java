package org.firstinspires.ftc.teamcode.drive;

/**
 * Standalone check (JDK 17, from the repository root):
 * javac -d /tmp/ftc-drive-check TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/MecanumMixer.java TeamCode/src/test/java/org/firstinspires/ftc/teamcode/drive/MecanumMixerCheck.java
 * java -cp /tmp/ftc-drive-check org.firstinspires.ftc.teamcode.drive.MecanumMixerCheck
 */
public final class MecanumMixerCheck {
    public static void main(String[] args) {
        check(new double[]{0, 0, 0, 0}, MecanumMixer.mix(0, 0, 0, 1));
        check(new double[]{1, 1, 1, 1}, MecanumMixer.mix(1, 0, 0, 1));
        check(new double[]{1, -1, -1, 1}, MecanumMixer.mix(0, 1, 0, 1));
        check(new double[]{1, -1, 1, -1}, MecanumMixer.mix(0, 0, 1, 1));
        check(new double[]{0.4, 0, 0, 0.4}, MecanumMixer.mix(1, 1, 0, 0.4));
        check(new double[]{0.2, 0.2, 0.2, 0.2}, MecanumMixer.mix(0.5, 0, 0, 0.4));
        check(new double[4], MecanumMixer.mix(Double.NaN, 0, 0, 1));
        check(new double[4], MecanumMixer.mix(0, Double.POSITIVE_INFINITY, 0, 1));
        check(new double[4], MecanumMixer.mix(1, 0, 0, -1));
        check(new double[]{1, 1, 1, 1}, MecanumMixer.mix(1, 0, 0, 2));
        // Every stick combination stays within the power cap and reverses symmetrically.
        for (int f = -10; f <= 10; f++) {
            for (int s = -10; s <= 10; s++) {
                for (int r = -10; r <= 10; r++) {
                    double[] powers = MecanumMixer.mix(f / 10.0, s / 10.0, r / 10.0, 0.4);
                    double[] reversed = MecanumMixer.mix(-f / 10.0, -s / 10.0, -r / 10.0, 0.4);
                    for (int i = 0; i < 4; i++) {
                        if (!Double.isFinite(powers[i]) || Math.abs(powers[i]) > 0.400000001
                                || Math.abs(powers[i] + reversed[i]) > 1e-9) {
                            throw new AssertionError("Power bounds or reversal failed");
                        }
                    }
                }
            }
        }
        System.out.println("Mecanum checks passed, including 9,261 stick combinations.");
    }

    private static void check(double[] expected, double[] actual) {
        for (int i = 0; i < expected.length; i++) {
            if (!Double.isFinite(actual[i]) || Math.abs(expected[i] - actual[i]) > 1e-9) {
                throw new AssertionError("Wheel " + i + ": expected " + expected[i] + ", got " + actual[i]);
            }
        }
    }
}
