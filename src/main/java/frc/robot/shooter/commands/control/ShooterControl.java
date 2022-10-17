package frc.robot.shooter.commands.control;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.shooter.ShooterBase;

public abstract class ShooterControl extends CommandBase {
    protected ShooterBase shooterSubsystem;
    public double outputPercent;

    public ShooterControl(ShooterBase shooter) {
        this.shooterSubsystem = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        periodic();
        shooterSubsystem.setPercentOutput(outputPercent);
    }

    public abstract void periodic();
}
