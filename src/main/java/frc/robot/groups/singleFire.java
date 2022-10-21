package frc.robot.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.shooter.ShooterBase;
import frc.robot.shooter.commands.SetShooter;
import frc.robot.shooter.commands.StopFlywheel;
import frc.robot.wrist.rollers.WristRollersBase;
import frc.robot.wrist.rollers.commands.SetRollers;
import frc.robot.wrist.rollers.commands.StopRollers;
import frc.robot.wrist.rotation.WristRotationBase;

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
