package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.function.LongSupplier;
import org.firstinspires.ftc.teamcode.config.TeleOpRumbleTuning;

/**
 * Shared toggle confirmation timing; does not track or suppress ordinary aim pulses.
 * CHANGES (2026-09-12): Bound aim suppression to the actual programmed confirmation steps.
 */
public final class ToggleRumble {
    private final Gamepad gamepad;
    private final LongSupplier clock;
    private long startedAtNanos;
    private long durationNanos;

    public ToggleRumble(Gamepad gamepad) {
        this(gamepad, System::nanoTime);
    }

    public ToggleRumble(Gamepad gamepad, LongSupplier clock) {
        this.gamepad = gamepad;
        this.clock = clock;
    }

    /** A new confirmation replaces any previous pattern, just like the SDK rumble queue. */
    public void play(boolean doublePulse) {
        Gamepad.RumbleEffect.Builder builder = new Gamepad.RumbleEffect.Builder()
                .addStep(TeleOpRumbleTuning.TOGGLE_STRENGTH, TeleOpRumbleTuning.TOGGLE_STRENGTH,
                        Math.max(0, TeleOpRumbleTuning.TOGGLE_STEP_MS));
        if (doublePulse) {
            builder.addStep(0, 0, Math.max(0, TeleOpRumbleTuning.TOGGLE_GAP_MS))
                    .addStep(TeleOpRumbleTuning.TOGGLE_STRENGTH, TeleOpRumbleTuning.TOGGLE_STRENGTH,
                            Math.max(0, TeleOpRumbleTuning.TOGGLE_STEP_MS));
        }
        Gamepad.RumbleEffect effect = builder.build();
        long durationMs = 0;
        for (Gamepad.RumbleEffect.Step step : effect.steps) durationMs += step.duration;
        gamepad.runRumbleEffect(effect);
        startedAtNanos = clock.getAsLong();
        durationNanos = durationMs * 1_000_000L;
    }

    /** Uses the programmed duration, not the SDK's estimate for all rumble sources. */
    public boolean isConfirming() {
        return durationNanos > 0 && clock.getAsLong() - startedAtNanos < durationNanos;
    }
}
