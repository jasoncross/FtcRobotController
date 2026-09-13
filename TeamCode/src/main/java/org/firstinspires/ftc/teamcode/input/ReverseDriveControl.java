package org.firstinspires.ftc.teamcode.input;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.utils.ToggleRumble;

/**
 * Driver perspective toggle: invert translation while preserving robot-relative twist.
 * CHANGES (2026-09-12): Restore left-stick toggle; require release if held at START and share timed confirmation feedback.
 */
public final class ReverseDriveControl {
    private boolean reversed;

    /** Construct at START: seed the button state without toggling or playing feedback. */
    public ReverseDriveControl(ControllerBindings bindings, Gamepad gamepad, ToggleRumble feedback) {
        bindings.bindPress(ControllerBindings.Pad.G1, ControllerBindings.Btn.L_STICK_BTN,
                gamepad.left_stick_button, () -> {
                    reversed = !reversed;
                    feedback.play(reversed);
                });
    }

    /** Apply only to forward/strafe input, never to manual or vision-generated twist. */
    public double translation(double value) {
        return reversed ? -value : value;
    }

    public boolean isReversed() {
        return reversed;
    }
}
