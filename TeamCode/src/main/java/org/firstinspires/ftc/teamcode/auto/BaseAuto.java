package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.drive.Drivebase;
import org.firstinspires.ftc.teamcode.vision.VisionAprilTag;
import org.firstinspires.ftc.teamcode.util.ObeliskSignal;

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
    protected VisionAprilTag vision;

    protected Drivebase drive;

    protected abstract Alliance alliance();
    protected abstract void runSequence() throws InterruptedException;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new Drivebase(this);

        // Optional vision for obelisk signal
        try {
            vision = new VisionAprilTag();
            vision.init(hardwareMap, "Webcam 1");
        } catch (Exception ignored) { vision = null; }

        // Prestart loop: show obelisk on FIRST LINE and let it latch in memory
        while (!isStarted() && !isStopRequested()) {
            if (vision != null) vision.observeObelisk();
            telemetry.addData("Obelisk", ObeliskSignal.getDisplay());
            telemetry.addData("Auto", "Alliance: %s", alliance());
            telemetry.update();
            sleep(20);
        }

        waitForStart();
        if (isStopRequested()) {
            if (vision != null) vision.stop();
            return;
        }

        runSequence();
        drive.stopAll();

        if (vision != null) vision.stop();

        telemetry.addLine("Auto complete – DS will queue TeleOp.");
        telemetry.update();
        sleep(750);
    }
