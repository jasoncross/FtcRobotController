package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.Alliance;

/** Blue alliance – Target start. */
public class Auto_Blue_Target extends BaseAuto {
    @Override protected Alliance alliance() { return Alliance.BLUE; }

    @Override
    protected void runSequence() throws InterruptedException {
        double startHeading = drive.heading();
        turnToGoalTag(1500);
        aimSpinUntilReady(2500);
        fireN(3);
        turnBackTo(startHeading);
        driveForwardInches(24.0);
    }
}
