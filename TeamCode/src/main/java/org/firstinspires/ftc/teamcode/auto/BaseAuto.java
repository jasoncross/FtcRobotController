package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.drive.Drivebase;

/*
 * FILE: BaseAuto.java
 * LOCATION: teamcode/.../auto/
 *
 * PURPOSE:
 * - Common autonomous scaffolding: initialize Drivebase, waitForStart, run a subclass sequence,
 *   cleanly stop, and display end-of-auto message (Driver Station will auto-load TeleOp).
 *
 * TUNABLES:
 * - None here; tune inside Drivebase or individual Auto subclasses.
 *
 * IMPORTANT FUNCTIONS:
 * - alliance(): each Auto returns RED/BLUE to share configs if needed.
 * - runSequence(): implement the actual path using move()/turn() and subsystems.
 */
public abstract class BaseAuto extends LinearOpMode {
    protected Drivebase drive;

    protected abstract Alliance alliance();
    protected abstract void runSequence() throws InterruptedException;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new Drivebase(this);

        telemetry.addData("Auto", "Alliance: %s", alliance());
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        runSequence();
        drive.stopAll();

        telemetry.addLine("Auto complete – DS will queue TeleOp.");
        telemetry.update();
        sleep(750);
    }
}
