package frc.robot.autos;

import org.talon540.vision.Limelight.LimelightVision;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.Measurements;
import frc.robot.drivetrain.DrivetrainBase;
import frc.robot.drivetrain.commands.CenterRobotOnHubStack;
import frc.robot.drivetrain.commands.DriveToDistance;
import frc.robot.shooter.ShooterBase;
import frc.robot.shooter.commands.SingleBallFire;
import frc.robot.wrist.WristBase;
import frc.robot.wrist.commands.rollers.KickupBall;

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
    public oldAuto(DrivetrainBase drivetrainBase, ShooterBase shooterBase, WristBase wristBase,
            LimelightVision limelightBase) {
        addCommands(
                new CenterRobotOnHubStack(drivetrainBase, limelightBase),

                new ParallelCommandGroup(
                        new DriveToDistance(drivetrainBase, limelightBase, 0), // TODO: find distance
                        new SingleBallFire(shooterBase)),

                new CenterRobotOnHubStack(drivetrainBase, limelightBase),

                new KickupBall(wristBase),

                new DriveToDistance(drivetrainBase, limelightBase,
                        Measurements.Robot.botlengthBumpersMeters + Measurements.Field.tarmacLengthMeters));
    }
}
