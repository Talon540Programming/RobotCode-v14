package frc.robot.drivetrain.commands;

import org.talon540.vision.Limelight.LimelightVision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Measurements;
import frc.robot.drivetrain.DrivetrainBase;

public class CenterRobotOnHubStack extends CommandBase {
    private DrivetrainBase driveBase;
    private LimelightVision limelightBase;

    private double ERROR = 15;
    private int centeredCount = 0;

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
        double nonZeroX = getNonZeroX(ERROR);

        double motorSpeed = (Math.abs(nonZeroX * .9)/59.6) + 0.05;
        motorSpeed = Math.round(motorSpeed * 100.0) / 100.0;

        // Uncomment for testing
        // motorSpeed = MathUtil.clamp(driveBase.rotationController.calculate(Math.toRadians(limelightBase.nonZeroX), 0), -1, 1);

        // if(!limelightBase.targetViewed) motorSpeed = 0.25;

        if(0 < nonZeroX) {
            driveBase.percentTankDrive(-motorSpeed, motorSpeed);
        } else if(nonZeroX < 0) {
            driveBase.percentTankDrive(motorSpeed, -motorSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        driveBase.brake();
        // limelightBase.disableLEDS();
        // driveBase.rotationController.reset();
    }

    @Override
    public boolean isFinished() {
        if(Measurements.Calculations.limelightCenteringDeadbandAngleDeg > Math.abs(getNonZeroX(1000000000))) {
            centeredCount++;
            return centeredCount == 7;
        } else {
            centeredCount = 0;
            return false;
        }

        // return Measurements.Calculations.limelightCenteringDeadbandAngleDeg > Math.abs(getNonZeroX(1000000000));
    }

    private double getNonZeroX() {
        return getNonZeroX(0);
    }

    private double getNonZeroX(double error) {
        return (limelightBase.nonZeroX == null || limelightBase.nonZeroX == 0) ? error : limelightBase.nonZeroX;
    }
}
