// ============================================================================
// FILE:    Auto_Red_Target.java
// LOCATION: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/
//
// PURPOSE:
//   Red-side TARGET start. Runs the shared autonomous sequence:
//     1) Turn to face the goal tag (AutoAim)
//     2) Spin flywheels to AutoSpeed
//     3) Fire 3 balls 3s apart
//     4) Turn back to original heading
//     5) Drive forward 24"
//
// NOTES:
//   - Goal Tag IDs live in vision/VisionAprilTag.java (Blue=20, Red=24).
//   - Shot spacing, RPM tolerance, and caps are centralized in
//       config/SharedRobotTuning.java  (was inline before).
//   - This OpMode will preselect "TeleOp - Red" after Auto.
// ============================================================================
package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Alliance;

@Autonomous(name="Auto: Red Target", group="Auto", preselectTeleOp="TeleOp - Red")
public class Auto_Red_Target extends BaseAuto {
    @Override protected Alliance alliance() { return Alliance.RED; }

    @Override
    protected void runSequence() throws InterruptedException {
        double startHeading = drive.heading();
        turnToGoalTag(1500);
        aimSpinUntilReady(2500);
        fireN(3);
        turnBackTo(startHeading);
        driveForwardInches(24.0);
        // BaseAuto handles StopAll() + vision shutdown in finally{}
    }
}
