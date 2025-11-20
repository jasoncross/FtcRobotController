/*
 * FILE: IntakeTuning.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/config/
 *
 * PURPOSE
 *   - Centralize every intake power/encoder threshold so TeleOp and Auto stay aligned
 *     as the hardware is retuned for the DECODE season ball column.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Intake power & driver defaults)
 *   - fillPower
 *       • Motor power while the shaft is freely filling (default mirrors the historical
 *         POWER_ON constant).
 *   - packingPower
 *       • Slightly reduced power once the first ball contacts the feed column so we can
 *         compress the stack without max current draw.
 *   - holdPower
 *       • Low power used during the SATURATED hold pulse to maintain positive pressure
 *         on the packed stack.
 *   - holdPulsePeriodMs / holdPulseOnMs
 *       • Defines the duty cycle for the SATURATED hold pulse. Set period ≤ 0 to run a
 *         steady holdPower instead of pulsing.
 *   - feedActiveHoldPower
 *       • Alternate hold power asserted whenever the Feed cycle is active so we are not
 *         adding load while launching.
 *   - sampleIntervalMs
 *       • Encoder sample cadence for classifying motion. Lower intervals react faster
 *         but increase noise.
 *   - freeDeltaTicks / contactDeltaTicks / stallDeltaTicks
 *       • Movement thresholds (ticks per sample) used to categorize "free flow",
 *         "packing", and "stall" behavior.
 *   - packingRangeTicks
 *       • Encoder travel allowed after first contact before the column is treated as
 *         fully packed (≈3 balls).
 *   - stallDebounceSamples
 *       • Number of consecutive stall-level samples required before declaring a true jam.
 *   - jamRecoveryPauseMs
 *       • Cooldown window applied after a jam before retrying forward flow.
 */
package org.firstinspires.ftc.teamcode.config;

public final class IntakeTuning {
    private IntakeTuning() {}

    // CHANGES (2025-11-16): Added inline comments for each tunable constant.

    public static double FILL_POWER = 0.9; // Motor power used while the intake is freely filling
    public static double PACKING_POWER = 0.75; // Reduced power once first contact occurs to compress the column
    public static double HOLD_POWER = 0.35; // Base power applied during the SATURATED hold pulse
    public static double FEED_ACTIVE_HOLD_POWER = 0.75; // Alternate hold power while the feed subsystem is cycling

    public static int HOLD_PULSE_PERIOD_MS = 400; // Total period (ms) for the SATURATED hold pulse cadence
    public static int HOLD_PULSE_ON_MS = 120; // Portion of the pulse period (ms) spent applying HOLD_POWER

    public static int SAMPLE_INTERVAL_MS = 50; // Encoder sampling cadence (ms) for movement classification
    public static int FREE_DELTA_TICKS = 20; // Minimum tick delta per sample to qualify as free-flow motion
    public static int CONTACT_DELTA_TICKS = 5; // Tick delta threshold indicating initial contact/packing behavior
    public static int STALL_DELTA_TICKS = 2; // Tick delta treated as a true stall when sustained
    public static int PACKING_RANGE_TICKS = 650; // Encoder travel allowed after first contact before saturation
    public static int STALL_DEBOUNCE_SAMPLES = 8; // Number of consecutive stall samples required before JAMMED state
    public static int JAM_RECOVERY_PAUSE_MS = 250; // Motor cooldown duration (ms) after declaring a jam
}
