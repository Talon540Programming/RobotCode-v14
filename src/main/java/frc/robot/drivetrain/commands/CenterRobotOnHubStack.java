package frc.robot.drivetrain.commands;

import org.talon540.vision.Limelight.LimelightVision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
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
        limelightBase.setPipeline(0);
        limelightBase.enableLEDS();
    }

    @Override
    public void execute() {
        double motorSpeed = (Math.abs(limelightBase.nonZeroX * .9)/59.6)+.05;
        motorSpeed = Math.round(motorSpeed * 100.0) / 100.0;

        // Uncomment for testing
        // motorSpeed = MathUtil.clamp(driveBase.rotationController.calculate(Math.toRadians(limelightBase.nonZeroX), 0), -1, 1);

        if(0 < limelightBase.nonZeroX) {
            driveBase.tankDrive(motorSpeed, -motorSpeed);
        } else if(limelightBase.nonZeroX < 0) {
            driveBase.tankDrive(-motorSpeed, motorSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        driveBase.brake();
    }

    @Override
    public boolean isFinished() {
        return Constants.deadbandAngle > Math.abs(limelightBase.nonZeroX);
    }

}
