package frc.robot.drivetrain.commands;

import org.talon540.sensors.vision.Limelight.LimelightVision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Measurements;
import frc.robot.drivetrain.DrivetrainBase;

public class DriveToDistance extends CommandBase {
    private LimelightVision limelightSubsystem;
    private DrivetrainBase driveSubsystem;

    private double distanceGoal, currentDistance;

    public DriveToDistance(DrivetrainBase drivetrainBase, LimelightVision limelightVisionBase, double desiredDistanceMeters) {
        this.limelightSubsystem = limelightVisionBase;
        this.driveSubsystem = drivetrainBase;

        this.distanceGoal = Math.abs(desiredDistanceMeters);

        addRequirements(drivetrainBase);
    }

    @Override
    public void initialize() {
        limelightSubsystem.enableLEDS();
    }

    @Override
    public void execute() {
        double percentL = 0;
        double percentR = 0;

        if (this.limelightSubsystem.targetViewed()) {
            currentDistance = this.limelightSubsystem.getDistanceFromTargetBase(Measurements.Field.upperHubHeightMeters);

            if (distanceGoal < currentDistance) {
                SmartDashboard.putString("Direction", "forward");
                // this.driveSubsystem.percentTankDrive(-0.5, -0.5);

                // percentL = -0.5;
                // percentR = -0.5;

                percentL = -MathUtil.clamp(Math.abs(currentDistance - distanceGoal), -0.5, 0.5);
                percentR = -MathUtil.clamp(Math.abs(currentDistance - distanceGoal), -0.5, 0.5);
            } else if (distanceGoal > currentDistance) {
                SmartDashboard.putString("Direction", "backwords");
                // this.driveSubsystem.percentTankDrive(0.25, 0.25);

                // percentL = 0.25;
                // percentR = 0.25;

                percentL = MathUtil.clamp(Math.abs(currentDistance - distanceGoal), -0.5, 0.5);
                percentR = MathUtil.clamp(Math.abs(currentDistance - distanceGoal), -0.5, 0.5);
            }

            this.driveSubsystem.percentTankDrive(
                // Math.copySign(Math.min(Math.abs(percentL), 0.5), percentL),
                // Math.copySign(Math.min(Math.abs(percentR), 0.5), percentR)
                percentL,
                percentR
            );

        } else {
            SmartDashboard.putString("Direction", "stopped");
            this.driveSubsystem.percentTankDrive(0, 0);
        }

        
    }

    @Override
    public void end(boolean interrupted) {
        this.driveSubsystem.brake();
    }

    private int count = 0;

    @Override
    public boolean isFinished() {
        if((distanceGoal - Measurements.Calculations.limelightDistanceOffsetMeters) <= currentDistance && currentDistance <= (distanceGoal + Measurements.Calculations.limelightDistanceOffsetMeters)) {
            count++;
            return count == 20;
        } else {
            count = 0;
            return false;
        }
    }

}
