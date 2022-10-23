package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autos.oldAuto;
import frc.robot.climberz.ClimberBase;
import frc.robot.constants.Measurements;
import frc.robot.constants.Flags.OperatorModes;
import frc.robot.drivetrain.DrivetrainBase;
import frc.robot.drivetrain.commands.CenterRobotOnHubStack;
import frc.robot.drivetrain.commands.DriveToDistance;
import frc.robot.groups.singleFire;
import frc.robot.shooter.ShooterBase;
import frc.robot.shooter.commands.control.*;
import frc.robot.drivetrain.commands.control.*;
import frc.robot.climberz.commands.control.*;
import frc.robot.wrist.rollers.commands.AutoIntake;
import frc.robot.wrist.rollers.commands.control.*;
import frc.robot.wrist.rotation.commands.MoveWristIn;
import frc.robot.wrist.rotation.commands.MoveWristOut;
import frc.robot.wrist.rotation.commands.control.*;
import frc.robot.wrist.rollers.WristRollersBase;
import frc.robot.wrist.rotation.WristRotationBase;

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
    private final ShooterBase shooterSubsystem = new ShooterBase();
    private final ClimberBase climberSubsystem = new ClimberBase();
    private final WristRotationBase rotationBase = new WristRotationBase();
    private final WristRollersBase rollersBase = new WristRollersBase();

    public RobotContainer() {
        gyro.calibrate();
        gyro.reset();

        configureButtonBindings();
        putTelemetrySendables();
    }

    /**
     * Configure button bindings for a control made based on the number of input controllers connected to the driverstation
     */
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

    /**
     * Configure button bindings based on a specific operator mode
     * @param operatorMode Desired operation mode (Single Driver Xbox, Single Driver Attack, Dual Driver Xbox and Attack)
     */
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
                this.rotationBase.setDefaultCommand(new XboxControllerWristRotationControl(rotationBase, xboxController));
                this.rollersBase.setDefaultCommand(new XboxControllerWristRollersControl(rollersBase, xboxController));
                /*
                 * ==========================
                 * Configure specific buttons
                 * ==========================
                 */

                // Center on hubs, preference on press once vs held
                // xboxController.buttons.LEFT_TRIGGER.whenPressed(new CenterRobotOnHubStack(drivetrainSubsystem, limelightSubsystem));
                xboxController.buttons.LEFT_TRIGGER.whenHeld(new CenterRobotOnHubStack(drivetrainSubsystem, limelightSubsystem));

                xboxController.buttons.LEFT_BUMPER.whenHeld(new DriveToDistance(drivetrainSubsystem, limelightSubsystem, Measurements.Calculations.optimalShootingRange));
                xboxController.buttons.RIGHT_BUMPER.whenHeld(new singleFire(shooterSubsystem, rotationBase, rollersBase));

                xboxController.buttons.START.whenPressed(new AutoIntake(rollersBase, rotationBase));

                xboxController.buttons.DPAD_EAST.whenPressed(new MoveWristOut(rotationBase));
                xboxController.buttons.DPAD_WEST.whenPressed(new MoveWristIn(rotationBase));

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
                this.rotationBase.setDefaultCommand(new AttackJoystickWristRotationControl(rotationBase, leftJoystick, rightJoystick));
                this.rollersBase.setDefaultCommand(new AttackJoystickWristRollersControl(rollersBase, leftJoystick, rightJoystick));

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
                this.rotationBase.setDefaultCommand(new XboxControllerWristRotationControl(rotationBase, xboxController));
                this.rollersBase.setDefaultCommand(new XboxControllerWristRollersControl(rollersBase, xboxController));

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
        return new oldAuto(drivetrainSubsystem, shooterSubsystem, rotationBase, rollersBase, limelightSubsystem);
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
     * Set up getters and senders for each telemetry object to the SmartDashboard update. 
     * Should be called in an initalization block, not periodic
     */
    private void putTelemetrySendables() {
        SmartDashboard.putData("Limelight", limelightSubsystem);
        SmartDashboard.putData("Flywheel", shooterSubsystem);
        SmartDashboard.putData("Drivetrain", drivetrainSubsystem);
        SmartDashboard.putData("Position Map", drivetrainSubsystem.positionMap);
        SmartDashboard.putData("Rollers Resistance Map", rollersBase.resistanceMap);
        SmartDashboard.putData("Rollers", rollersBase);
        SmartDashboard.putData("Rotation", rotationBase);
    }

    public void updateSmartDashboard() {
        SmartDashboard.putNumber("Distance From Hubstack", limelightSubsystem.getDistanceFromTargetBase(Measurements.Field.upperHubHeightMeters));
    }

    /**
     * Used to notify people that some kind of error has occured
     */
    public void reportError() {
        limelightSubsystem.blinkLEDS();
    }
}
