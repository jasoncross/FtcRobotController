// ============================================================================
// FILE:    Auto_Blue_Target.java
// LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/
//
// PURPOSE:
//   Blue-side TARGET start. Runs the shared autonomous sequence.
//
// NOTES:
//   - Tunables centralized in config/SharedRobotTuning.java.
//   - Preselects "TeleOp - Blue" at end of Auto.
// ============================================================================
package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Alliance;

@Autonomous(name="Auto: Blue Target", group="Auto", preselectTeleOp="TeleOp - Blue")
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
