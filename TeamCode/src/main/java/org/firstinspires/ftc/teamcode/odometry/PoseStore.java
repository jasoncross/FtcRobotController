package org.firstinspires.ftc.teamcode.odometry;

/*
 * FILE: PoseStore.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/odometry/
 *
 * PURPOSE
 *   - Provide a static handoff container so Autonomous can persist the final
 *     fused pose for TeleOp to read during INIT, keeping odometry continuous
 *     across the mode transition.
 *
 * CHANGES (2025-11-26): Exposed explicit set/get/has/consume helpers so Auto and
 *                        TeleOp can coordinate handoff status and emit
 *                        telemetry when resuming from an Auto pose.
 * CHANGES (2025-11-25): Introduced shared pose memory for Auto→TeleOp seeding
 *                        and tag-based re-localization fallbacks.
 */
public final class PoseStore {

    private static FieldPose lastPose = null;

    private PoseStore() { /* no instances */ }

    /** Store a copy of the fused pose for later retrieval. */
    public static synchronized void setLastKnownPose(FieldPose pose) {
        if (pose == null) return;
        lastPose = pose.copy();
    }

    /** Returns a copy of the last known pose without clearing it. */
    public static synchronized FieldPose getLastKnownPose() {
        return (lastPose == null) ? null : lastPose.copy();
    }

    /** Indicates whether a pose has been stored for handoff. */
    public static synchronized boolean hasLastKnownPose() {
        return lastPose != null;
    }

    /**
     * Returns the stored pose and clears the container so TeleOp only seeds
     * once unless Auto or other code refreshes the value.
     */
    public static synchronized FieldPose consumeLastKnownPose() {
        FieldPose out = getLastKnownPose();
        lastPose = null;
        return out;
    }

    /** Clears any stored pose without returning it. */
    public static synchronized void clear() {
        lastPose = null;
    }

    /** Legacy alias maintained for older callers. */
    public static void save(FieldPose pose) { setLastKnownPose(pose); }

    /** Legacy alias maintained for older callers. */
    public static FieldPose consume() { return consumeLastKnownPose(); }
}
