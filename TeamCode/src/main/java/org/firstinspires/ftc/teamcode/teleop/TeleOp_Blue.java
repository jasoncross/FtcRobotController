package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Alliance;

/*
 * FILE: TeleOp_Blue.java
 * LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleop/
 *
 * PURPOSE
 *   - Register the BLUE alliance TeleOp entry while reusing all behavior from
 *     TeleOpAllianceBase.
 *
 * TUNABLE PARAMETERS
 *   - None locally; all tuning lives in TeleOpAllianceBase and shared config files.
 *
 * METHODS
 *   - alliance()
 *       • Returns Alliance.BLUE so the base class selects blue tags and telemetry.
 *
 * NOTES
 *   - Appears on the Driver Station menu as "TeleOp - Blue" (see @TeleOp annotation).
 */
@TeleOp(name="TeleOp - Blue", group="TeleOp")
public class TeleOp_Blue extends TeleOpAllianceBase {
    @Override protected Alliance alliance() { return Alliance.BLUE; }
}
