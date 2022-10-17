package frc.robot.drivetrain.commands;

import org.talon540.vision.Limelight.LimelightVision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Measurements;
import frc.robot.drivetrain.DrivetrainBase;

public class CenterRobotOnHubStack extends CommandBase {
    private DrivetrainBase driveBase;
    private LimelightVision limelightBase;

    public CenterRobotOnHubStack(DrivetrainBase dBase, LimelightVision lBase) {
        this.driveBase = dBase;
        this.limelightBase = lBase;

        addRequirements(dBase, lBase);
    }

    @Override
    public void initialize() {
        limelightBase.enableLEDS();
    }

    @Override
    public void execute() {
        double nonZeroX = limelightBase.nonZeroX == null ? 15 : limelightBase.nonZeroX;

        double motorSpeed = (Math.abs(nonZeroX * .9)/59.6)+.05;
        motorSpeed = Math.round(motorSpeed * 100.0) / 100.0;

        // Uncomment for testing
        // motorSpeed = MathUtil.clamp(driveBase.rotationController.calculate(Math.toRadians(limelightBase.nonZeroX), 0), -1, 1);

        if(0 < nonZeroX) {
            driveBase.percentTankDrive(-motorSpeed, motorSpeed);
        } else if(nonZeroX < 0) {
            driveBase.percentTankDrive(motorSpeed, -motorSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        limelightBase.disableLEDS();
        driveBase.brake();
        // driveBase.rotationController.reset();
    }

    @Override
    public boolean isFinished() {
        return Measurements.Calculations.limelightCenteringDeadbandAngleDeg > Math.abs(limelightBase.nonZeroX);
    }

}
