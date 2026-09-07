package org.firstinspires.ftc.teamcode.input;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.junit.Test;
import static org.junit.Assert.*;
import static org.firstinspires.ftc.teamcode.input.ControllerBindings.*;

public class ControllerBindingsTest {
    @Test public void pressAndHoldCoexistAndOnlyPressRetriggersAfterRelease() {
        Gamepad g1 = new Gamepad(), g2 = new Gamepad();
        int[] counts = {0, 0};
        ControllerBindings bindings = new ControllerBindings()
                .bindPress(Pad.G1, Btn.A, () -> counts[0]++)
                .bindHold(Pad.G1, Btn.A, () -> counts[1]++);
        g1.a = true;
        bindings.update(g1, g2);
        bindings.update(g1, g2);
        assertArrayEquals(new int[]{1, 2}, counts);
        g1.a = false; bindings.update(g1, g2);
        g1.a = true; bindings.update(g1, g2);
        assertArrayEquals(new int[]{2, 3}, counts);
        bindings.clear(); bindings.update(g1, g2);
        assertArrayEquals(new int[]{2, 3}, counts);
    }

    @Test public void triggerAndBumperAreIndependentAndGamepadsDoNotCross() {
        Gamepad g1 = new Gamepad(), g2 = new Gamepad();
        int[] counts = {0, 0};
        ControllerBindings bindings = new ControllerBindings()
                .bindTriggerPress(Pad.G2, Trigger.RT, () -> counts[0]++)
                .bindPress(Pad.G2, Btn.RB, () -> counts[1]++);
        g1.right_trigger = 1; g1.right_bumper = true; bindings.update(g1, g2);
        assertArrayEquals(new int[]{0, 0}, counts);
        g2.right_trigger = 0.6f; bindings.update(g1, g2); bindings.update(g1, g2);
        assertArrayEquals(new int[]{1, 0}, counts);
        g2.right_bumper = true; bindings.update(g1, g2);
        assertArrayEquals(new int[]{1, 1}, counts);
    }

    @Test public void toggleOnlyChangesOnRisingEdges() {
        Gamepad g1 = new Gamepad(), g2 = new Gamepad();
        int[] counts = {0, 0};
        ControllerBindings bindings = new ControllerBindings()
                .bindToggle(Pad.G1, Btn.Y, () -> counts[0]++, () -> counts[1]++);
        g1.y = true; bindings.update(g1, g2); bindings.update(g1, g2);
        g1.y = false; bindings.update(g1, g2);
        g1.y = true; bindings.update(g1, g2);
        assertArrayEquals(new int[]{1, 1}, counts);
    }
}
