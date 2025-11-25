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
 * CHANGES (2025-11-25): Introduced shared pose memory for Auto→TeleOp seeding
 *                        and tag-based re-localization fallbacks.
 */
public final class PoseStore {

    private static FieldPose lastPose = null;

    private PoseStore() { /* no instances */ }

    public static void save(FieldPose pose) {
        if (pose == null) return;
        lastPose = pose.copy();
    }

    public static FieldPose consume() {
        FieldPose out = (lastPose == null) ? null : lastPose.copy();
        lastPose = null;
        return out;
    }
}
