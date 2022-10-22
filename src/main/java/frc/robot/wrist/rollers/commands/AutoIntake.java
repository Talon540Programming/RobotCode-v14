package frc.robot.wrist.rollers.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.wrist.rollers.WristRollersBase;

public class AutoIntake extends SequentialCommandGroup {
    public AutoIntake(WristRollersBase rollersBase) {
        addCommands(
            new RollUntilIntake(rollersBase),
            new WaitCommand(1),
            new StopRollers(rollersBase)
        );
    }
}
