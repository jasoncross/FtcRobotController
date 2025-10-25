package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
 * FILE: Intake.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/
 *
 * PURPOSE
 *   - Wrap the intake motor with simple on/off controls shared by TeleOp,
 *     Autonomous, and StopAll safety hooks.
 *   - Track current state so toggle buttons and auto assists stay synchronized.
 *
 * TUNABLE PARAMETERS (SEE TunableDirectory.md → Intake power & driver defaults)
 *   - powerOn
 *       • Motor power while the intake is active.
 *       • Increase toward 0.9 when ARTIFACTS slip; reduce to ~0.6 if jams occur.
 *         TeleOpAllianceBase can tweak DEFAULT_INTAKE_ENABLED but not this value.
 *
 * METHODS
 *   - toggle() / set(boolean)
 *       • Flip or directly command the intake state.
 *   - isOn()
 *       • Used by telemetry + gating logic.
 *   - stop()
 *       • Force the intake off (StopAll, autonomous failsafes).
 *
 * NOTES
 *   - TeleOpAllianceBase.DEFAULT_INTAKE_ENABLED controls whether TeleOp starts
 *     with the intake running—adjust there for driver preference.
 */
public class Intake {
    private final DcMotorEx motor;
    private boolean on = false;
    public double powerOn = 0.8; // Shared intake power; referenced by TeleOp + Auto assists (see TunableDirectory)

    public Intake(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, "Intake");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /** Toggle intake state (useful for button bindings). */
    public void toggle() { set(!on); }

    /** Set intake to run or stop at the configured power. */
    public void set(boolean enable) {
        on = enable;
        motor.setPower(on ? powerOn : 0);
    }

    /** @return true when intake motor is currently running. */
    public boolean isOn() { return on; }

    /** Immediately turns intake OFF (safe to call repeatedly). */
    public void stop() {
        set(false);
    }
}
