// ============================================================================
// FILE:    Auto_Red_Human.java
// LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/
//
// PURPOSE:
//   Red-side HUMAN start. Runs the shared autonomous sequence (see Target file).
//
// NOTES:
//   - Tunables centralized in config/SharedRobotTuning.java.
//   - Preselects "TeleOp - Red" at end of Auto.
// ============================================================================
package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Alliance;

@Autonomous(name="Auto: Red Human", group="Auto", preselectTeleOp="TeleOp - Red")
public class Auto_Red_Human extends BaseAuto {
    @Override protected Alliance alliance() { return Alliance.RED; }

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
