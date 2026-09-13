package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.config.TeleOpRumbleTuning;
import org.junit.Test;
import static org.junit.Assert.*;

/**
 * Verifies confirmation content and timing independently of ordinary aim rumble.
 * CHANGES (2026-09-12): Cover configured pulse strength/duration, deadline boundaries and replacement.
 */
public class ToggleRumbleTest {
    private static class FeedbackGamepad extends Gamepad {
        RumbleEffect last;
        @Override public void runRumbleEffect(RumbleEffect effect) { last = effect; }
        @Override public boolean isRumbling() { return true; } // Ordinary aim rumble may still be active.
    }

    @Test public void configuredStepsAndDeadlineDoNotDependOnOtherRumble() {
        double strength = TeleOpRumbleTuning.TOGGLE_STRENGTH;
        int step = TeleOpRumbleTuning.TOGGLE_STEP_MS, gap = TeleOpRumbleTuning.TOGGLE_GAP_MS;
        try {
            TeleOpRumbleTuning.TOGGLE_STRENGTH = 0.4;
            TeleOpRumbleTuning.TOGGLE_STEP_MS = 73;
            TeleOpRumbleTuning.TOGGLE_GAP_MS = 31;
            FeedbackGamepad gamepad = new FeedbackGamepad();
            long[] clock = {1_000_000_000L};
            ToggleRumble feedback = new ToggleRumble(gamepad, () -> clock[0]);
            assertFalse(feedback.isConfirming());
            feedback.play(true);
            assertEquals(3, gamepad.last.steps.size());
            assertStep(gamepad.last.steps.get(0), 102, 73);
            assertStep(gamepad.last.steps.get(1), 0, 31);
            assertStep(gamepad.last.steps.get(2), 102, 73);
            clock[0] += 176_999_999L;
            assertTrue(feedback.isConfirming());
            clock[0]++;
            assertFalse(feedback.isConfirming());
            assertTrue(gamepad.isRumbling());
            feedback.play(true);
            clock[0] += 10_000_000L;
            feedback.play(false); // Shorter new pattern replaces the previous deadline.
            assertEquals(1, gamepad.last.steps.size());
            assertStep(gamepad.last.steps.get(0), 102, 73);
            clock[0] += 72_999_999L;
            assertTrue(feedback.isConfirming());
            clock[0]++;
            assertFalse(feedback.isConfirming());
        } finally {
            TeleOpRumbleTuning.TOGGLE_STRENGTH = strength;
            TeleOpRumbleTuning.TOGGLE_STEP_MS = step;
            TeleOpRumbleTuning.TOGGLE_GAP_MS = gap;
        }
    }

    private static void assertStep(Gamepad.RumbleEffect.Step step, int strength, int durationMs) {
        assertEquals(strength, step.large);
        assertEquals(strength, step.small);
        assertEquals(durationMs, step.duration);
    }
}
