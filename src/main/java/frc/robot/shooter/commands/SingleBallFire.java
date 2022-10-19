package frc.robot.shooter.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.shooter.ShooterBase;

/**
 * Prep the flywheel to fire a single ball at max velocity
 * @implNote disables flywheel after {@code 4 seconds}
 */
public class SingleBallFire extends SequentialCommandGroup {
    public SingleBallFire(ShooterBase shooterBase) {
        addCommands(
            new SetShooter(shooterBase, 1),
            new WaitCommand(4),
            new StopFlywheel(shooterBase)
        );
    }
}
