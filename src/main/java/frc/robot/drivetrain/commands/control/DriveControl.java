package frc.robot.drivetrain.commands.control;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drivetrain.DrivetrainBase;

public abstract class DriveControl extends CommandBase {
    DrivetrainBase drivetrainBase;

    /**
     * Proportional term for drivetrain control manipulation within [-1,1]
     * 
     * @implNote must be set to 0 when there is no trottle so the bot will stop
     */
    protected double kleftDrive, krightDrive;

    DriveControl(DrivetrainBase dBase) {
        this.drivetrainBase = dBase;
        addRequirements(dBase);
    }

    @Override
    public void execute() {
        drivetrainBase.tankDrive(kleftDrive, krightDrive);
    }
}
