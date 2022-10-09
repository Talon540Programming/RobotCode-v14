package frc.robot.drivetrain.commands;

import org.talon540.vision.Limelight.LimelightVision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.drivetrain.DrivetrainBase;

public class DriveToDistance extends CommandBase {
    private LimelightVision limelightSubsystem;
    private DrivetrainBase driveSubsystem;

    private double distanceGoal;

    private boolean finished = false;

    DriveToDistance(DrivetrainBase drivetrainBase, LimelightVision limelightVisionBase, double desiredDistance) {
        this.limelightSubsystem = limelightVisionBase;
        this.driveSubsystem = drivetrainBase;

        this.distanceGoal = Math.abs(desiredDistance);

        addRequirements(drivetrainBase, limelightVisionBase);
    }

    @Override
    public void execute() {
        if (this.limelightSubsystem.targetViewed) {
            double currentDistance = this.limelightSubsystem
                    .getDistanceFromTargetBase(Constants.FieldData.upperHubHeightMeters);

            if (distanceGoal < currentDistance) {
                this.driveSubsystem.tankDrive(0.5, 0.5);
            } else if (distanceGoal > currentDistance) {
                this.driveSubsystem.tankDrive(-0.25, -0.25);
            } else if (distanceGoal - Constants.driveToDistanceOffset <= currentDistance
                    && currentDistance <= distanceGoal + Constants.driveToDistanceOffset) {
                this.finished = true;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.driveSubsystem.brake();
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

}
