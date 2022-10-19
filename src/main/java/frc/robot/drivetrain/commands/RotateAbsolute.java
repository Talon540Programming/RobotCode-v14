package frc.robot.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drivetrain.DrivetrainBase;

public class RotateAbsolute extends CommandBase {
    protected DrivetrainBase drivetrainBase;
    protected double targetSetpoint;

    public RotateAbsolute(double targetAngle, DrivetrainBase drivetrainBase) {
        this.targetSetpoint = targetAngle % 360.0;
        this.drivetrainBase = drivetrainBase;
    }

    @Override
    public void execute() {
        drivetrainBase.rotateToAngle(targetSetpoint);
    }

    @Override
    public boolean isFinished() {
        return drivetrainBase.rotationController.atSetpoint();
    }
}
