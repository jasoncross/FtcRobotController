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
 *       - Obelisk AprilTag signal (Tags 21/22/23) is RESET at Auto start,
 *         observed in prestart, and continuously latched via Vision's own
 *         background poller during Auto so late sightings are not missed.
 *       - A lightweight HUD thread refreshes FIRST-LINE telemetry during Auto
 *         so operators can see updates while the sequence is running.
 *
 * NOTES:
 *   - Preserves prior functionality: Drivebase init, runSequence() call,
 *     and end-of-run drive.stopAll(). No autonomous behavior removed.
 *   - Vision is optional; if init fails, Auto continues without it.
 *   - The HUD thread only writes telemetry; all motion logic remains unchanged.
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

    // ----------------------- HUD (telemetry refresher) -----------------------
    private volatile boolean hudRunning = false;
    private Thread hudThread = null;

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

        // --------------------- Enable vision auto-latch + HUD -----------------
        if (vision != null) vision.setObeliskAutoLatchEnabled(true);
        startHud();

        // --------------------- Autonomous Sequence ---------------------------
        try {
            runSequence();
        } finally {
            // --------------------- Cleanup / Safety --------------------------
            stopHud();
            drive.stopAll(); // make sure all motion/mechanisms are stopped
            stopVisionIfAny(); // stop camera cleanly
        }

        telemetry.addLine("Auto complete – DS will queue TeleOp.");
        telemetry.update();
        sleep(750);
    }

    // ----------------------- Helpers ----------------------------------------
    protected final void stopVisionIfAny() {
        try {
            if (vision != null) {
                vision.setObeliskAutoLatchEnabled(false);
                vision.stop();
            }
        } catch (Exception ignored) { /* ignore */ }
    }

    /** Starts a lightweight HUD that refreshes FIRST-LINE telemetry during Auto. */
    protected final void startHud() {
        if (hudRunning) return;
        hudRunning = true;
        hudThread = new Thread(() -> {
            try {
                while (hudRunning && !isStopRequested()) {
                    try {
                        telemetry.addData("Obelisk", ObeliskSignal.getDisplay());
                        telemetry.addData("Auto", "Alliance: %s", alliance());
                        telemetry.update();
                    } catch (Exception ignored) { /* best effort */ }
                    try { Thread.sleep(200); } catch (InterruptedException ie) { break; }
                }
            } finally {
                hudRunning = false;
            }
        }, "AutoHUD");
        hudThread.setDaemon(true);
        hudThread.start();
    }

    /** Stops the HUD thread. */
    protected final void stopHud() {
        hudRunning = false;
        if (hudThread != null) {
            try { hudThread.join(150); } catch (InterruptedException ignored) {}
            hudThread = null;
        }
    }
}
