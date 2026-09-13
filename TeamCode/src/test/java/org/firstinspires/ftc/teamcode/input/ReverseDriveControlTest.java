package org.firstinspires.ftc.teamcode.input;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.config.ControllerTuning;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.drive.MecanumMixer;
import org.firstinspires.ftc.teamcode.utils.ToggleRumble;
import org.junit.Test;
import static org.junit.Assert.*;

/**
 * Checks the restored driver perspective across press edges and combined drive commands.
 * CHANGES (2026-09-12): Cover held-at-START release, reverse edges, feedback, translation, twist and production trigger scaling.
 */
public class ReverseDriveControlTest {
    private static class FeedbackGamepad extends Gamepad {
        int effects;
        RumbleEffect lastEffect;
        @Override public void runRumbleEffect(RumbleEffect effect) {
            effects++;
            lastEffect = effect;
        }
    }

    @Test public void leftStickTogglesOncePerPressAndNewRunStartsNormal() {
        FeedbackGamepad g1 = new FeedbackGamepad();
        Gamepad g2 = new Gamepad();
        ControllerBindings bindings = new ControllerBindings();
        ReverseDriveControl control = new ReverseDriveControl(bindings, g1, new ToggleRumble(g1));
        assertFalse(control.isReversed());
        g2.left_stick_button = true;
        bindings.update(g1, g2);
        assertFalse(control.isReversed());
        assertEquals(0, g1.effects);
        g1.left_stick_button = true;
        bindings.update(g1, g2);
        bindings.update(g1, g2);
        assertTrue(control.isReversed());
        assertEquals(1, g1.effects);
        assertEquals(3, g1.lastEffect.steps.size()); // pulse, gap, pulse
        g1.left_stick_button = false;
        bindings.update(g1, g2);
        assertTrue(control.isReversed());
        g1.left_stick_button = true;
        bindings.update(g1, g2);
        assertFalse(control.isReversed());
        assertEquals(2, g1.effects);
        assertEquals(1, g1.lastEffect.steps.size());
        assertFalse(new ReverseDriveControl(new ControllerBindings(), g1, new ToggleRumble(g1)).isReversed());
    }

    @Test public void heldAtStartRequiresReleaseAndNextRunDoesNotRetainReverse() {
        FeedbackGamepad g1 = new FeedbackGamepad();
        Gamepad g2 = new Gamepad();
        g1.left_stick_button = true; // Already held when START is pressed.
        ControllerBindings bindings = new ControllerBindings();
        ReverseDriveControl first = new ReverseDriveControl(bindings, g1, new ToggleRumble(g1));
        bindings.update(g1, g2);
        bindings.update(g1, g2);
        assertFalse(first.isReversed());
        assertEquals(0, g1.effects);
        g1.left_stick_button = false;
        bindings.update(g1, g2);
        g1.left_stick_button = true;
        bindings.update(g1, g2);
        assertTrue(first.isReversed());
        bindings.clear(); // End of run; next run constructs the production control again.
        ControllerBindings nextBindings = new ControllerBindings();
        ReverseDriveControl next = new ReverseDriveControl(nextBindings, g1, new ToggleRumble(g1));
        nextBindings.update(g1, g2);
        assertFalse(next.isReversed());
        assertEquals(1, g1.effects);
        g1.left_stick_button = false;
        nextBindings.update(g1, g2);
        g1.left_stick_button = true;
        nextBindings.update(g1, g2);
        assertTrue(next.isReversed());
        assertEquals(2, g1.effects);
    }

    @Test public void reverseTranslationPreservesTwistAndSlowPowerCap() {
        FeedbackGamepad g1 = new FeedbackGamepad();
        ControllerBindings bindings = new ControllerBindings();
        ReverseDriveControl control = new ReverseDriveControl(bindings, g1, new ToggleRumble(g1));
        assertEquals(0.7, control.translation(0.7), 0);
        g1.left_stick_button = true;
        bindings.update(g1, new Gamepad());
        assertEquals(-0.7, control.translation(0.7), 0);
        assertEquals(0.4, control.translation(-0.4), 0);
        double cap = RobotConfig.DRIVE_POWER_LIMIT;
        double[] normalTurn = MecanumMixer.mix(0, 0, 0.6, cap);
        assertArrayEquals(normalTurn, MecanumMixer.mix(control.translation(0),
                control.translation(0), 0.6, cap), 1e-12);
        double[] full = MecanumMixer.mix(control.translation(0.8), control.translation(0.5), 0.3, cap);
        double[] slow = MecanumMixer.mix(control.translation(0.8), control.translation(0.5), 0.3,
                cap * DriveInputScaling.slowScale(1));
        assertArrayEquals(MecanumMixer.mix(-0.8, -0.5, 0.3, cap), full, 1e-12);
        for (int i = 0; i < full.length; i++) {
            assertEquals(full[i] * ControllerTuning.SLOWEST_SPEED, slow[i], 1e-12);
            assertTrue(Math.abs(slow[i]) <= cap * ControllerTuning.SLOWEST_SPEED + 1e-12);
        }
    }
}
