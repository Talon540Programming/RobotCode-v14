package frc.robot.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drivetrain.DrivetrainBase;

public class SlowDriveBack extends CommandBase{
    private DrivetrainBase drivetrainBase;

    public SlowDriveBack(DrivetrainBase drivetrainBase) {
        this.drivetrainBase = drivetrainBase;
        addRequirements(drivetrainBase);
    }

    @Override
    public void execute() {
        // drivetrainBase.driveStraight(0);
        drivetrainBase.percentTankDrive(0.10, 0.10);
    }
}
