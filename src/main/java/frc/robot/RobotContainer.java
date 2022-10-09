package frc.robot;

import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.climberz.ClimberBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Flags.OperatorModes;
import frc.robot.drivetrain.DrivetrainBase;
import frc.robot.drivetrain.commands.CenterRobotOnHubStack;
import frc.robot.drivetrain.commands.drive.AttackJoystickDrive;
import frc.robot.drivetrain.commands.drive.XboxControllerDrive;
import frc.robot.shooter.ShooterBase;
import frc.robot.wrist.WristBase;

import org.talon540.control.TalonJoystick;
import org.talon540.control.TalonXboxController;
import org.talon540.vision.Limelight.LimelightVision;

import com.kauailabs.navx.frc.AHRS;

public class RobotContainer {
    // Devices
    private final TalonJoystick leftJoystick = new TalonJoystick(0);
    private final TalonJoystick rightJoystick = new TalonJoystick(1);
    private final TalonXboxController xboxController = new TalonXboxController(2);

    private final AHRS gyro = new AHRS(Port.kUSB);

    // Subsystems
    public final LimelightVision limelightSubsystem = new LimelightVision(Constants.RobotData.RobotMeasurement.LimelightAngleDegrees, Constants.RobotData.RobotMeasurement.LimelightHeightMeters);
    private final DrivetrainBase drivetrainSubsystem = new DrivetrainBase(gyro);
    private final WristBase wristSubsystem = new WristBase();
    private final ShooterBase shooterSubsystem = new ShooterBase();
    private final ClimberBase climberSubsystem = new ClimberBase();

    public RobotContainer() {
        gyro.calibrate();
        gyro.reset();

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // TODO: Add logic to get choice from smart dashboard
        configureButtonBindings(OperatorModes.XBOX_AND_ATTACK);
    }

    private void configureButtonBindings(OperatorModes operatorMode) {
        switch(operatorMode) {
            case XBOX_ONLY:
                // Configure Default Commands
                this.drivetrainSubsystem.setDefaultCommand(new XboxControllerDrive(drivetrainSubsystem, xboxController));

                // Configure specific buttons

                // Center on hubs, preference on press once vs held
                xboxController.buttons.RIGHT_TRIGGER.whenPressed(new CenterRobotOnHubStack(drivetrainSubsystem, limelightSubsystem));
                // xboxController.buttons.RIGHT_TRIGGER.whenHeld(new CenterRobotOnHubStack(drivetrainSubsystem, limelightSubsystem));

                break;
            case ATTACK_ONLY:
                // Configure Default Commands
                this.drivetrainSubsystem.setDefaultCommand(new AttackJoystickDrive(drivetrainSubsystem, leftJoystick, rightJoystick));

                // Configure specific buttons

                rightJoystick.buttons.TRIGGER.whenPressed(new CenterRobotOnHubStack(drivetrainSubsystem, limelightSubsystem));
                // rightJoystick.buttons.TRIGGER.whenHeld(new CenterRobotOnHubStack(drivetrainSubsystem, limelightSubsystem));

                break;
            case XBOX_AND_ATTACK:
                // Configure Default Commands
                this.drivetrainSubsystem.setDefaultCommand(new AttackJoystickDrive(drivetrainSubsystem, leftJoystick, rightJoystick));

                // Configure specific buttons

                // Center on hubs, preference on press once vs held
                xboxController.buttons.RIGHT_TRIGGER.whenPressed(new CenterRobotOnHubStack(drivetrainSubsystem, limelightSubsystem));
                // xboxController.buttons.RIGHT_TRIGGER.whenHeld(new CenterRobotOnHubStack(drivetrainSubsystem, limelightSubsystem));
                // rightJoystick.buttons.TRIGGER.whenPressed(new CenterRobotOnHubStack(drivetrainSubsystem, limelightSubsystem));
                // rightJoystick.buttons.TRIGGER.whenHeld(new CenterRobotOnHubStack(drivetrainSubsystem, limelightSubsystem));


                break;
        }
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
