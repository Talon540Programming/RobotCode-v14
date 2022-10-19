package frc.robot.autos;

import org.talon540.vision.Limelight.LimelightVision;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.Measurements;
import frc.robot.drivetrain.DrivetrainBase;
import frc.robot.drivetrain.commands.CenterRobotOnHubStack;
import frc.robot.drivetrain.commands.DriveToDistance;
import frc.robot.shooter.ShooterBase;
import frc.robot.shooter.commands.SetShooter;
import frc.robot.shooter.commands.StopFlywheel;
import frc.robot.wrist.WristBase;
import frc.robot.wrist.commands.rollers.KickupBall;
import frc.robot.wrist.commands.rollers.SetRollers;
import frc.robot.wrist.commands.rollers.StopRollers;

/**
 * Old Auto Sequence. Auto sequence used during the 2022, Rapid React
 * Competition
 * <ul>
 * <li>Center on the hub
 * <li>Drive to a certain distance from the hub
 * <li>Rev Flywheel to target velocity for some x time
 * <li>Run the rollers for some x time
 * <li>Shoot
 * <li>Disable shooter and roller motors
 * <li>Drive a certain distance away from the hub
 * </ul>
 */
public class oldAuto extends SequentialCommandGroup {
    public oldAuto(DrivetrainBase drivetrainBase, ShooterBase shooterBase, WristBase wristBase, LimelightVision limelightBase) {
        addCommands(
            // Center the robot on the hub stack
            new CenterRobotOnHubStack(drivetrainBase, limelightBase),

            // Get to a good distance from the hubstack and rev the flywheel
            new ParallelCommandGroup(
                    new DriveToDistance(drivetrainBase, limelightBase, 1.3), // TODO: find distance
                    new SequentialCommandGroup(
                        new SetShooter(shooterBase, 1),
                        new WaitCommand(2)
                    )
            ),

            // Center the robot again to account for any error that occured during driving
            new CenterRobotOnHubStack(drivetrainBase, limelightBase),

            // Fire the ball from the trough and stop the flywheel and rollers after
            new SequentialCommandGroup(
                new SequentialCommandGroup(
                    new SetRollers(wristBase, 0.5),
                    new WaitCommand(3)
                ),

                new StopFlywheel(shooterBase),
                new StopRollers(wristBase)
            ),

            // Taxi
            new DriveToDistance(drivetrainBase, limelightBase, Measurements.Robot.botlengthBumpersMeters + Measurements.Field.tarmacLengthMeters));
    }
}
