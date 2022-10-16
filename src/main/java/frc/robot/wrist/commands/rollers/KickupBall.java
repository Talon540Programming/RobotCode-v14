package frc.robot.wrist.commands.rollers;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.wrist.WristBase;

/**
 * Run the rollers to kickup any ball currently in the trough into the flywheel
 * @implNote disables rollers after {@code 4 seconds}
 */
public class KickupBall extends SequentialCommandGroup {
    private WristBase wristBase;

    public KickupBall(WristBase wristBase) {
        this.wristBase = wristBase;
        addCommands(
            new SetRollerPercent(wristBase, 0.5),
            new WaitCommand(4)
        );
    }

    @Override
    public void end(boolean interrupted) {
        this.wristBase.stopRollers();
    }
}
