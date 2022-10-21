package frc.robot.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.shooter.ShooterBase;
import frc.robot.shooter.commands.SetShooter;
import frc.robot.shooter.commands.StopFlywheel;
import frc.robot.wrist.WristRollersBase;
import frc.robot.wrist.WristRotationBase;
import frc.robot.wrist.commands.rollers.SetRollers;
import frc.robot.wrist.commands.rollers.StopRollers;

public class singleFire extends SequentialCommandGroup {
    public singleFire(ShooterBase shooterBase, WristRotationBase rotationBase, WristRollersBase rollersBase) {
        addCommands(
            new SetShooter(shooterBase, 1),
            new WaitCommand(2),
            new SetRollers(rollersBase, -0.5),
            new WaitCommand(3),
            new StopFlywheel(shooterBase),
            new StopRollers(rollersBase)
        );
    }
}
