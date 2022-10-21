package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autos.oldAuto;
import frc.robot.climberz.ClimberBase;
import frc.robot.climberz.commands.control.AttackJoystickClimberzControl;
import frc.robot.climberz.commands.control.XboxControllerClimberzControl;
import frc.robot.constants.Measurements;
import frc.robot.constants.Flags.OperatorModes;
import frc.robot.drivetrain.DrivetrainBase;
import frc.robot.drivetrain.commands.CenterRobotOnHubStack;
import frc.robot.drivetrain.commands.DriveToDistance;
import frc.robot.drivetrain.commands.control.AttackJoystickDriveControl;
import frc.robot.drivetrain.commands.control.XboxControllerDriveControl;
import frc.robot.groups.singleFire;
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
    private final LimelightVision limelightSubsystem = new LimelightVision(
        Measurements.Robot.LimelightAngleDegrees,
        Measurements.Robot.LimelightHeightMeters
        // 99.0/39.37
    );
    private final DrivetrainBase drivetrainSubsystem = new DrivetrainBase(gyro);
    private final WristBase wristSubsystem = new WristBase();
    private final ShooterBase shooterSubsystem = new ShooterBase();
    private final ClimberBase climberSubsystem = new ClimberBase();

    public RobotContainer() {
        gyro.calibrate();
        gyro.reset();

        // configureButtonBindings(OperatorModes.ATTACK_ONLY);
        // configureButtonBindings(OperatorModes.XBOX_ONLY);
        configureButtonBindings();

        putTelemetrySendables();
    }

    private void configureButtonBindings() {
        boolean joystickOneConnected = DriverStation.isJoystickConnected(0);
        boolean joystickTwoConnected = DriverStation.isJoystickConnected(1);
        boolean xboxcontrollerConnected = DriverStation.isJoystickConnected(2);

        if((joystickOneConnected && joystickTwoConnected)) {
            if(xboxcontrollerConnected) {
                configureButtonBindings(OperatorModes.XBOX_AND_ATTACK);
            } else {
                configureButtonBindings(OperatorModes.ATTACK_ONLY);
            }
        } else {
            configureButtonBindings(OperatorModes.XBOX_ONLY);
        }

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
                xboxController.buttons.LEFT_TRIGGER.whenPressed(new CenterRobotOnHubStack(drivetrainSubsystem, limelightSubsystem));
                // xboxController.buttons.LEFT_TRIGGER.whenHeld(new CenterRobotOnHubStack(drivetrainSubsystem, limelightSubsystem));

                xboxController.buttons.LEFT_BUMPER.whenHeld(new DriveToDistance(drivetrainSubsystem, limelightSubsystem, 1.3));
                xboxController.buttons.RIGHT_BUMPER.whenPressed(new singleFire(shooterSubsystem, wristSubsystem));

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
        return new oldAuto(drivetrainSubsystem, shooterSubsystem, wristSubsystem, limelightSubsystem);
    }

    /**
     * Disable all lights on the robot
     */
    public void disableAllLights() {
        disableFunctionalLights();
    }

    /**
     * Disables lights used for calculations and computer vision
     */
    public void disableFunctionalLights() {
        this.limelightSubsystem.disableLEDS();
    }

    /**
     * Enable all lights on the robot
     */
    public void enableAllLights() {
        enableFunctionalLights();
    }

    /**
     * Enables lights used for calculations and computer vision
     */
    public void enableFunctionalLights() {
        this.limelightSubsystem.enableLEDS();
    }

    /**
     * Put data to the SmartDashboard from the subsystems
     * Should be called in an initalization block
     */
    private void putTelemetrySendables() {
        SmartDashboard.putData("Limelight", limelightSubsystem);
        SmartDashboard.putData("Flywheel", shooterSubsystem);
        SmartDashboard.putData("Drivetrain", drivetrainSubsystem);
        SmartDashboard.putData("Position Map", drivetrainSubsystem.positionMap);
    }

    public void updateSmartDashboard() {
        SmartDashboard.putNumber("DISTANCE", limelightSubsystem.getDistanceFromTargetBase(Measurements.Field.upperHubHeightMeters));
    }

    /**
     * Used to notify people that some kind of error has occured
     */
    public void reportError() {
        limelightSubsystem.blinkLEDS();
    }
}
