package frc.robot.shooter.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.shooter.ShooterBase;

public class SingleBallFire extends SequentialCommandGroup {
    private ShooterBase shooterBase;

    public SingleBallFire(ShooterBase shooterBase) {
        this.shooterBase = shooterBase;
        addCommands(
            new SetShooterSpeed(shooterBase, 1),
            new WaitCommand(6)
        );
    }

    @Override
    public void end(boolean interrupted) {
        this.shooterBase.stopFlywheel();
    }
}
