package frc.robot.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.shooter.ShooterBase;

public class StopFlywheel extends CommandBase {
    private ShooterBase shooterBase;

    public StopFlywheel(ShooterBase sBase) {
        this.shooterBase = sBase;
    }

    @Override
    public void initialize() {
        shooterBase.stopFlywheel();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
