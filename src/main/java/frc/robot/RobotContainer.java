package frc.robot;

import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.climberz.ClimberBase;
import frc.robot.climberz.commands.control.AttackJoystickClimberzControl;
import frc.robot.climberz.commands.control.XboxControllerClimberzControl;
import frc.robot.constants.Constants;
import frc.robot.constants.Flags.OperatorModes;
import frc.robot.drivetrain.DrivetrainBase;
import frc.robot.drivetrain.commands.CenterRobotOnHubStack;
import frc.robot.drivetrain.commands.control.AttackJoystickDriveControl;
import frc.robot.drivetrain.commands.control.XboxControllerDriveControl;
import frc.robot.shooter.ShooterBase;
import frc.robot.shooter.commands.control.AttackJoystickShooterControl;
import frc.robot.shooter.commands.control.XboxControllerShooterContol;
import frc.robot.wrist.WristBase;
import frc.robot.wrist.commands.control.AttackJoystickWristControl;
import frc.robot.wrist.commands.control.XboxControllerWristControl;

import org.talon540.control.AttackJoystick.TalonJoystick;
import org.talon540.control.XboxController.TalonXboxController;
import org.talon540.vision.Limelight.LimelightVision;

import com.kauailabs.navx.frc.AHRS;

public class RobotContainer {
    // Devices
    private final TalonJoystick leftJoystick = new TalonJoystick(0);
    private final TalonJoystick rightJoystick = new TalonJoystick(1);
    private final TalonXboxController xboxController = new TalonXboxController(2);

    private final AHRS gyro = new AHRS(Port.kUSB);

    // Subsystems
    public final LimelightVision limelightSubsystem = new LimelightVision(
        Constants.RobotData.RobotMeasurement.LimelightAngleDegrees,
        Constants.RobotData.RobotMeasurement.LimelightHeightMeters
    );
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
                /*
                 * ==========================
                 * Configure Default Commands
                 * ==========================
                 */
                this.drivetrainSubsystem.setDefaultCommand(new XboxControllerDriveControl(drivetrainSubsystem, xboxController));
                this.climberSubsystem.setDefaultCommand(new XboxControllerClimberzControl(climberSubsystem, xboxController));
                this.shooterSubsystem.setDefaultCommand(new XboxControllerShooterContol(shooterSubsystem, xboxController));
                this.wristSubsystem.setDefaultCommand(new XboxControllerWristControl(wristSubsystem, xboxController));

                /*
                 * ==========================
                 * Configure specific buttons
                 * ==========================
                 */

                // Center on hubs, preference on press once vs held
                // xboxController.buttons.LEFT_TRIGGER.whenPressed(new CenterRobotOnHubStack(drivetrainSubsystem, limelightSubsystem));
                xboxController.buttons.LEFT_TRIGGER.whenHeld(new CenterRobotOnHubStack(drivetrainSubsystem, limelightSubsystem));

                break;
            case ATTACK_ONLY:
                /*
                 * ==========================
                 * Configure Default Commands
                 * ==========================
                 */
                this.drivetrainSubsystem.setDefaultCommand(new AttackJoystickDriveControl(drivetrainSubsystem, leftJoystick, rightJoystick));
                this.climberSubsystem.setDefaultCommand(new AttackJoystickClimberzControl(climberSubsystem, leftJoystick, rightJoystick));
                this.shooterSubsystem.setDefaultCommand(new AttackJoystickShooterControl(shooterSubsystem, leftJoystick, rightJoystick));
                this.wristSubsystem.setDefaultCommand(new AttackJoystickWristControl(wristSubsystem, leftJoystick, rightJoystick));

                /*
                 * ==========================
                 * Configure specific buttons
                 * ==========================
                 */

                // leftJoystick.buttons.TRIGGER.whenPressed(new CenterRobotOnHubStack(drivetrainSubsystem, limelightSubsystem));
                leftJoystick.buttons.TRIGGER.whenHeld(new CenterRobotOnHubStack(drivetrainSubsystem, limelightSubsystem));

                break;
            case XBOX_AND_ATTACK:
                /*
                 * ==========================
                 * Configure Default Commands
                 * ==========================
                 */
                this.drivetrainSubsystem.setDefaultCommand(new AttackJoystickDriveControl(drivetrainSubsystem, leftJoystick, rightJoystick));
                this.climberSubsystem.setDefaultCommand(new XboxControllerClimberzControl(climberSubsystem, xboxController));
                this.shooterSubsystem.setDefaultCommand(new XboxControllerShooterContol(shooterSubsystem, xboxController));
                this.wristSubsystem.setDefaultCommand(new XboxControllerWristControl(wristSubsystem, xboxController));

                /*
                 * ==========================
                 * Configure specific buttons
                 * ==========================
                 */

                // Center on hubs, preference on press once vs held
                // xboxController.buttons.LEFT_TRIGGER.whenPressed(new CenterRobotOnHubStack(drivetrainSubsystem, limelightSubsystem));
                xboxController.buttons.LEFT_TRIGGER.whenHeld(new CenterRobotOnHubStack(drivetrainSubsystem, limelightSubsystem));
                // leftJoystick.buttons.TRIGGER.whenPressed(new CenterRobotOnHubStack(drivetrainSubsystem, limelightSubsystem));
                // leftJoystick.buttons.TRIGGER.whenHeld(new CenterRobotOnHubStack(drivetrainSubsystem, limelightSubsystem));


                break;
        }
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
