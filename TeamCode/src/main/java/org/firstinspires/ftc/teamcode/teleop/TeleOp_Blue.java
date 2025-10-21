package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Alliance;

/*
 * FILE: TeleOp_Blue.java
 * PURPOSE:
 * - Blue alliance TeleOp entry; inherits behavior from TeleOpAllianceBase.
 * TUNABLES: none (inherits).
 * IMPORTANT: Shows up on DS as "TeleOp - Blue".
 */
@TeleOp(name="TeleOp - Blue", group="TeleOp")
public class TeleOp_Blue extends TeleOpAllianceBase {
    @Override protected Alliance alliance() { return Alliance.BLUE; }
}
