package frc.robot.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.shooter.ShooterBase;

public class SetShooterSpeed extends CommandBase {
    private ShooterBase shooterBase;
    private double percent;

    public SetShooterSpeed(ShooterBase shooterBase, double percentOut) {
        this.shooterBase = shooterBase;
        this.percent = percentOut;
        addRequirements(shooterBase);
    }

    @Override
    public void initialize() {
        shooterBase.setPercentOutput(percent);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
