package org.firstinspires.ftc.teamcode.input;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.config.TeleOpRumbleTuning;

/**
 * Driver perspective toggle: invert translation while preserving robot-relative twist.
 * CHANGES (2026-09-12): Restore left-stick toggle and single/double rumble confirmation.
 */
public final class ReverseDriveControl {
    private boolean reversed;

    /** A new instance starts in normal drive; ControllerBindings owns press-edge detection. */
    public ReverseDriveControl(ControllerBindings bindings, Gamepad gamepad) {
        bindings.bindPress(ControllerBindings.Pad.G1, ControllerBindings.Btn.L_STICK_BTN, () -> {
            reversed = !reversed;
            Gamepad.RumbleEffect.Builder effect = new Gamepad.RumbleEffect.Builder()
                    .addStep(TeleOpRumbleTuning.TOGGLE_STRENGTH, TeleOpRumbleTuning.TOGGLE_STRENGTH,
                            TeleOpRumbleTuning.TOGGLE_STEP_MS);
            if (reversed) {
                effect.addStep(0, 0, TeleOpRumbleTuning.TOGGLE_GAP_MS)
                        .addStep(TeleOpRumbleTuning.TOGGLE_STRENGTH, TeleOpRumbleTuning.TOGGLE_STRENGTH,
                                TeleOpRumbleTuning.TOGGLE_STEP_MS);
            }
            gamepad.runRumbleEffect(effect.build());
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
