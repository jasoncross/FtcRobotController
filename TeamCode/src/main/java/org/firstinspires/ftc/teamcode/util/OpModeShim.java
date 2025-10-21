package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/*
 * FILE: OpModeShim.java
 * LOCATION: teamcode/.../util/
 *
 * PURPOSE:
 * - Allows TeleOp (which extends OpMode) to reuse classes that expect a LinearOpMode
 *   (e.g., Drivebase needing opModeIsActive()/idle()) by wrapping the OpMode.
 *
 * TUNABLES:
 * - None. This is plumbing only.
 *
 * IMPORTANT:
 * - Use new OpModeShim(this) inside TeleOp.init() and pass it to Drivebase, etc.
 * - This shim is safe for TeleOp use; it always reports "active".
 */
public class OpModeShim extends LinearOpMode {
    private final OpMode op;
    public OpModeShim(OpMode op) { this.op = op; }

    @Override public void runOpMode() {}
    @Override public boolean opModeIsActive() { return true; }
    @Override public boolean isStarted() { return true; }
    @Override public boolean isStopRequested() { return false; }
    @Override public void requestOpModeStop() {}
    @Override public void sleep(long ms) { try { Thread.sleep(ms); } catch (InterruptedException ignored) {} }

    @Override public com.qualcomm.robotcore.hardware.HardwareMap hardwareMap() { return op.hardwareMap; }
    @Override public com.qualcomm.robotcore.external.Telemetry telemetry() { return op.telemetry; }
}
