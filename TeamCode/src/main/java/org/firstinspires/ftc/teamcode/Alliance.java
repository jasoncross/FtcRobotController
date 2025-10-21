package org.firstinspires.ftc.teamcode;

/*
 * FILE: Alliance.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
 *
 * PURPOSE:
 * - Simple enum to indicate the robot's current alliance (RED or BLUE).
 * - Used by TeleOp and Autonomous OpModes to pick alliance-specific behavior
 *   (e.g., AprilTag IDs, mirrored paths, DS "preselectTeleOp" target).
 *
 * TUNABLES (WHAT YOU MAY NEED TO CHANGE):
 * - None. This file intentionally has no tunables.
 *
 * IMPORTANT CLASSES/FUNCTIONS:
 * - enum Alliance { RED, BLUE }
 *   Use: pass Alliance.RED / Alliance.BLUE into config helpers or store
 *   it inside your OpModes to choose the correct settings for that side.
 *
 * NOTES:
 * - Keep this file tiny and importable from anywhere—it's referenced by many
 *   subsystems and OpModes.
 */
public enum Alliance {
    RED, BLUE
}
