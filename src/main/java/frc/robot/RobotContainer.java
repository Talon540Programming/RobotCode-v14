package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.climberz.ClimberBase;
import frc.robot.drivetrain.DrivetrainBase;
import frc.robot.drivetrain.commands.AttackJoystickDrive;
import frc.robot.drivetrain.commands.XboxControllerDrive;
import frc.robot.shooter.ShooterBase;
import frc.robot.wrist.WristBase;

import org.talon540.control.TalonJoystick;
import org.talon540.control.TalonXboxController;
import org.talon540.vision.Limelight.LimelightVision;

public class RobotContainer {
    // Subsystems
    private final LimelightVision limelightSubsystem = new LimelightVision(Constants.RobotData.RobotMeasurement.LimelightAngleDegrees, Constants.RobotData.RobotMeasurement.LimelightHeightMeters);
    private final DrivetrainBase drivetrainSubsystem = new DrivetrainBase();
    private final WristBase wristSubsystem = new WristBase();
    private final ShooterBase shooterSubsystem = new ShooterBase();
    private final ClimberBase climberSubsystem = new ClimberBase();

    // Devices
    private final TalonJoystick leftJoystick = new TalonJoystick(0);
    private final TalonJoystick rightJoystick = new TalonJoystick(1);
    private final TalonXboxController xboxController = new TalonXboxController(2);

    public RobotContainer() {
        this.drivetrainSubsystem.setDefaultCommand(new AttackJoystickDrive(drivetrainSubsystem, leftJoystick, rightJoystick));
        // this.drivetrainSubsystem.setDefaultCommand(new XboxControllerDrive(drivetrainSubsystem, xboxController));

        configureButtonBindings();
    }

    private void configureButtonBindings() {}

    public Command getAutonomousCommand() {
        return null;
    }
}