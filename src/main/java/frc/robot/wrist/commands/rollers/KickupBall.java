package frc.robot.wrist.commands.rollers;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.wrist.WristBase;

/**
 * Run the rollers to kickup any ball currently in the trough into the flywheel
 * @implNote disables rollers {@code 4 seconds} after the command has been scheduled
 */
public class KickupBall extends SequentialCommandGroup {

    public KickupBall(WristBase wristBase) {
        addCommands(
            new SetRollers(wristBase, 0.5),
            new WaitCommand(3),
            new StopRollers(wristBase)
        );
    }
}
