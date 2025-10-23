package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.drive.Drivebase;
import org.firstinspires.ftc.teamcode.utils.ObeliskSignal;
import org.firstinspires.ftc.teamcode.vision.VisionAprilTag;

/*
 * ============================================================================
 * FILE: BaseAuto.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/
 *
 * PURPOSE:
 *   Abstract base class for all DECODE season Autonomous OpModes.
 *   Provides common setup/teardown for the drivetrain and (optionally) vision,
 *   ensures mechanisms are made safe at the end, and exposes a simple template:
 *
 *       - Subclasses implement alliance() and runSequence().
 *       - VisionAprilTag is started best-effort and stopped automatically.
 *       - The Obelisk AprilTag signal (Tags 21/22/23) is RESET at Auto start,
 *         observed during the prestart loop, and continuously polled in a
 *         lightweight background watcher thread while Auto runs.
 *         Once latched, it persists into TeleOp (we do NOT clear it there).
 *
 * NOTES:
 *   - Preserves prior functionality: Drivebase init, runSequence() call,
 *     and end-of-run drive.stopAll(). No autonomous behavior removed.
 *   - Vision is optional; if init fails, Auto continues without it.
 *   - Telemetry is updated on the main thread only (watcher does NOT touch it).
 *
 * METHODS (for subclasses to implement/override):
 *   - Alliance alliance(): which side are we running?
 *   - void runSequence() throws InterruptedException: main autonomous steps.
 *   - (Optional) void onPreStartLoop(): hook for additional prestart telemetry.
 * ============================================================================
 */
public abstract class BaseAuto extends LinearOpMode {

    // ----------------------- Shared Subsystems -------------------------------
    protected Drivebase drive;
    protected VisionAprilTag vision; // optional; may be null if camera absent

    // ----------------------- Obelisk Watcher --------------------------------
    private volatile boolean obeliskWatcherRunning = false;
    private Thread obeliskWatcherThread = null;

    // ----------------------- Template Methods --------------------------------
    /** Return the alliance (RED/BLUE) for this Auto variant. */
    protected abstract Alliance alliance();

    /** Implement the autonomous path/sequence here. */
    protected abstract void runSequence() throws InterruptedException;

    /** Optional hook for subclasses to add prestart telemetry/logic. */
    protected void onPreStartLoop() { /* no-op by default */ }

    // ----------------------- OpMode Lifecycle --------------------------------
    @Override
    public void runOpMode() throws InterruptedException {
        // Drivetrain
        drive = new Drivebase(this);

        // Vision (optional – keep Auto robust even if no camera/driver issue)
        try {
            vision = new VisionAprilTag();
            vision.init(hardwareMap, "Webcam 1");
        } catch (Exception ex) {
            vision = null;
        }

        // --------------------- Reset Obelisk for this Auto --------------------
        // Requirement: At the start of AUTO, the observed value should be reset
        // until seen. It may then persist into TeleOp (we do not clear later).
        ObeliskSignal.clear();

        // --------------------- Pre-Start Loop --------------------------------
        // Let the robot sit on the field and watch the obelisk before start.
        while (!isStarted() && !isStopRequested()) {
            if (vision != null) {
                // Latch obelisk order (Tags 21/22/23) into shared memory
                vision.observeObelisk();
            }

            // FIRST LINE: show the currently latched obelisk order (or ---)
            telemetry.addData("Obelisk", ObeliskSignal.getDisplay());
            telemetry.addData("Auto", "Alliance: %s", alliance());

            // Let subclass add extra prestart info without spamming new lines
            onPreStartLoop();

            telemetry.update();
            sleep(20);
        }

        // Safety check
        if (isStopRequested()) {
            stopVisionIfAny();
            return;
        }

        // --------------------- Start background watcher ----------------------
        startObeliskWatcher();

        // --------------------- Autonomous Sequence ---------------------------
        try {
            runSequence();
        } finally {
            // ensure watcher is stopped even if runSequence throws
            stopObeliskWatcher();
        }

        // --------------------- Cleanup / Safety ------------------------------
        drive.stopAll();       // make sure all motion/mechanisms are stopped
        stopVisionIfAny();     // stop camera cleanly

        telemetry.addLine("Auto complete – DS will queue TeleOp.");
        telemetry.update();
        sleep(750);
    }

    // ----------------------- Helpers ----------------------------------------
    protected final void stopVisionIfAny() {
        try {
            if (vision != null) vision.stop();
        } catch (Exception ignored) { /* ignore */ }
    }

    /** Starts a lightweight background watcher that polls obelisk tags. */
    protected final void startObeliskWatcher() {
        if (vision == null || obeliskWatcherRunning) return;
        obeliskWatcherRunning = true;
        obeliskWatcherThread = new Thread(() -> {
            try {
                while (obeliskWatcherRunning && opModeIsActive() && !isStopRequested()) {
                    try {
                        vision.observeObelisk(); // just latches to ObeliskSignal
                    } catch (Exception ignored) { /* keep trying */ }
                    try {
                        Thread.sleep(75); // ~13 Hz light polling
                    } catch (InterruptedException ie) {
                        break;
                    }
                }
            } finally {
                obeliskWatcherRunning = false;
            }
        }, "ObeliskWatcher");
        obeliskWatcherThread.setDaemon(true);
        obeliskWatcherThread.start();
    }

    /** Stops the background obelisk watcher thread. */
    protected final void stopObeliskWatcher() {
        obeliskWatcherRunning = false;
        if (obeliskWatcherThread != null) {
            try {
                obeliskWatcherThread.join(150);
            } catch (InterruptedException ignored) { }
            obeliskWatcherThread = null;
        }
    }
}
