package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
 * FILE: Intake.java
 * LOCATION: teamcode/.../subsystems/
 *
 * PURPOSE:
 * - Simple on/off intake control for motor "Intake".
 *
 * TUNABLES:
 * - powerOn: intake running power (0.5–1.0 typical).
 *
 * IMPORTANT FUNCTIONS:
 * - toggle(): flips intake state.
 * - set(boolean): explicitly sets intake on/off.
 * - isOn(): returns current state.
 * - stop(): immediately turns intake OFF (used by TeleOp StopAll).
 *
 * NOTES:
 * - NEW: Startup ON/OFF is now controlled by TeleOp via DEFAULT_INTAKE_ENABLED.
 * - NEW (2025-10-23): Added stop() for integration with StopAll.
 */
public class Intake {
    private final DcMotorEx motor;
    private boolean on = false;
    public double powerOn = 0.8;

    public Intake(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, "Intake");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void toggle() { set(!on); }

    public void set(boolean enable) {
        on = enable;
        motor.setPower(on ? powerOn : 0);
    }

    public boolean isOn() { return on; }

    /** Immediately turns intake OFF (safe to call repeatedly). */
    public void stop() {
        set(false);
    }
}
