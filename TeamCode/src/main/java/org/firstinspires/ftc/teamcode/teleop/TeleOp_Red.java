package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Alliance;

/*
 * FILE: TeleOp_Red.java
 * PURPOSE:
 * - Red alliance TeleOp entry; inherits all behavior from TeleOpAllianceBase.
 * TUNABLES: none (inherits).
 * IMPORTANT: Shows up on DS as "TeleOp - Red".
 */
@TeleOp(name="TeleOp - Red", group="TeleOp")
public class TeleOp_Red extends TeleOpAllianceBase {
    @Override protected Alliance alliance() { return Alliance.RED; }
}
