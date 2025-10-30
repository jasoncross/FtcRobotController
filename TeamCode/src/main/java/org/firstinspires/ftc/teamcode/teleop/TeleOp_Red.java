package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Alliance;

/*
 * FILE: TeleOp_Red.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleop/
 *
 * PURPOSE
 *   - Register the RED alliance TeleOp entry while delegating behavior to
 *     TeleOpAllianceBase.
 *
 * TUNABLE PARAMETERS
 *   - None locally; refer to TeleOpAllianceBase and shared configs.
 *
 * METHODS
 *   - alliance()
 *       • Returns Alliance.RED so the base class mirrors goal tags and telemetry.
 *
 * NOTES
 *   - Appears on the Driver Station menu as "TeleOp - Red".
 */
@TeleOp(name="TeleOp - Red", group="TeleOp")
public class TeleOp_Red extends TeleOpAllianceBase {
    @Override protected Alliance alliance() { return Alliance.RED; }
}
