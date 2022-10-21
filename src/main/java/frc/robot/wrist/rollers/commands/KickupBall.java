package frc.robot.wrist.rollers.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.wrist.rollers.WristRollersBase;

/**
 * Run the rollers to kickup any ball currently in the trough into the flywheel
 * @implNote disables rollers {@code 4 seconds} after the command has been scheduled
 */
public class KickupBall extends SequentialCommandGroup {

    public KickupBall(WristRollersBase rollersBase) {
        addCommands(
            new SetRollers(rollersBase, 0.5),
            new WaitCommand(3),
            new StopRollers(rollersBase)
        );
    }
}
