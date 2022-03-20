package frc.robot.Modules;

import frc.robot.Robot;
import frc.robot.Modules.RobotInformation.RobotData.MotorData.motorTypes.Motors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


public class MotorControl {
/**
 * Given the ideal velocity and the transfer percentage, return the RPM
 *
 * @param idealVelocity the velocity you want to achieve in meters per second
 * @param transferPercent the percentage of the ideal velocity that is transferred to the wheel.
 * @return The RPM of the motor.
 */
    public static double getRPM(Motors wantedMotor) {
        switch(wantedMotor) {
            case Shooter:
                double shooterVelocity = getCurrentVelocity(Robot.shooterFly);
                return shooterVelocity/RobotInformation.RobotData.MotorData.Shooter.Flywheel.gearRatio/2048*600;   
            case Wrist:
                double wristVelocity = getCurrentVelocity(Robot.wrist);
                return wristVelocity/RobotInformation.RobotData.MotorData.Intake.Wrist.gearRatio/2048*600;
            case Extension:
                double extensionVelocity = getCurrentVelocity(Robot.climbExtension);
                return extensionVelocity/RobotInformation.RobotData.MotorData.Climbers.ClimbExtension.gearRatio/2048*600;
            case Rotation:
                double rotationVelocity = getCurrentVelocity(Robot.climbRotation);
                return rotationVelocity/RobotInformation.RobotData.MotorData.Climbers.ClimbRotation.gearRatio/2048*600;
            default:
                return 0;
        }
    }

/**
 * This function returns the current velocity of the motor. The motor must be a Falcon500
 *
 * @param motor The motor you want to get the velocity of.
 * @return The integrated sensor velocity of the motor.
 */
    public static double getCurrentVelocity(WPI_TalonFX motor) { // If using a TalonFX motor only
        return (motor.getSensorCollection().getIntegratedSensorVelocity());
    }

/**
 * This function returns the current position of the motor. The motor must be a Falcon500
 *
 * @param motor The motor you want to get the current position of.
 * @return The current position of the motor.
 */
    public static double getCurrentPosition(WPI_TalonFX motor) { // If using a TalonFX motor only
        return (motor.getSensorCollection().getIntegratedSensorAbsolutePosition());
    }

/**
 * Initializes the motors for the robot
 */
    public static void motor_init() {
        Robot.rightMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 40, .5));
        Robot.rightMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 40, .5));
        Robot.rightSlave.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 40, .5));
        Robot.rightSlave.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 40, .5));
        Robot.leftMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 40, .5));
        Robot.leftMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 40, .5));
        Robot.leftSlave.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 40, .5));
        Robot.leftSlave.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 40, .5));

        // Initialize Climber's and their Motor Brakes
        Robot.climbRotation.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 80, 80, .5));
        Robot.climbExtension.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 80, 80, .5));
        Robot.climbRotation.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 80, 80, .5));
        Robot.climbExtension.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 80, 80, .5));
        Robot.climbRotation.setNeutralMode(NeutralMode.Brake);
        Robot.climbExtension.setNeutralMode(NeutralMode.Brake);

        // Configure Shooter Flywheel
        Robot.shooterFly.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 1); // Tells to use in-built falcon 500 sensor
        Robot.shooterFly.configNominalOutputForward(0, 1);
        Robot.shooterFly.configNominalOutputReverse(0, 1);
        Robot.shooterFly.configPeakOutputForward(1, 1);
        Robot.shooterFly.configPeakOutputReverse(-1, 1);
        Robot.shooterFly.setInverted(false);
        Robot.shooterFly.setSensorPhase(false);
    }

    public static class FlywheelCode {
        /**
         * If the right bumper is pressed, run the flywheel at full power.
         */
        public static void flywheel() {
            if(Robot.controller.getRightBumper()) {
                Robot.shooterFly.set(ControlMode.PercentOutput, 1);
            } else {
                Robot.shooterFly.set(ControlMode.PercentOutput, 0);
            }
        }

    }
    
/**
 * This class is used to control the drivetrain of the robot
 */
    public static class DriveCode {
        /** Main Drive Call */
        public static void tankDrive() {
            if ((Math.abs(Robot.rightJoy.getY()) > 0.2) || Math.abs(Robot.leftJoy.getY()) > 0.2) {
                Robot.drive.tankDrive((-Robot.leftJoy.getY() * RobotInformation.DriveTeamInfo.driverPercentage), ( Robot.rightJoy.getY()* RobotInformation.DriveTeamInfo.driverPercentage));
            }
        }

        // Use negative number to move back and positive number to move forward
        //TODO: adjust kP to drive nice and smooth- if it takes to long just use the traditional moveBack and pray.java drivetrain is even
        public static void driveStraight(double power) {
            double kP = 0.0027;
            double correction = -Robot.gyro.getAngle();
            double turnPower = kP*correction;
            Robot.drive.arcadeDrive(power, turnPower, false);
        }

        /**
         * Drive the robot using the left and right master motors at the given motor speeds.*
         *
         * @param motorSpeedLeft The speed that you want the left motor to go.
         * @param motorSpeedRight The speed that you want the right side to go.
         */
        public static void oldDriveTrain(double motorSpeedLeft, double motorSpeedRight) {
            Robot.leftMaster.set(ControlMode.PercentOutput, motorSpeedLeft);
            Robot.rightMaster.set(ControlMode.PercentOutput, -motorSpeedRight);
        }
    }
}
