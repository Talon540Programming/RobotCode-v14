package frc.robot.wrist.rollers.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.wrist.rollers.WristRollersBase;
import frc.robot.wrist.rotation.WristRotationBase;
import frc.robot.wrist.rotation.commands.MoveWristOut;

public class AutoIntake extends SequentialCommandGroup {
    public AutoIntake(WristRollersBase rollersBase, WristRotationBase rotationBase) {
        addCommands(
            new MoveWristOut(rotationBase),
            new RollUntilIntake(rollersBase),
            new WaitCommand(1),
            new StopRollers(rollersBase)
        );
    }
}
