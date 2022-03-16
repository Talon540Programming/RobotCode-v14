package frc.robot.Modules;

import frc.robot.Robot;

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
    public static double getRPM(double idealVelocity, double transferPercent) {
        return (((idealVelocity*(1/transferPercent))/(Math.PI*0.1016))*60); // rudimentary calculation that's 90% wrong
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
 * If the right bumper is pressed, run the flywheel at full power.
 */
    public static void flywheel() {
        if(Robot.controller.getRightBumper()) {
            Robot.shooterFly.set(ControlMode.PercentOutput, 1);
        } else {
            Robot.shooterFly.set(ControlMode.PercentOutput, 0);
        }
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

        Robot.rightMaster.setInverted(true);

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


/**
 * This class is used to control the drivetrain of the robot
 */
    public static class DriveCode {
        /** Main Drive Call */
        public static void tankDrive() {
            if ((Math.abs(Robot.rightJoy.getY()) > 0.2) || Math.abs(Robot.leftJoy.getY()) > 0.2) {
                Robot.drive.tankDrive((Robot.leftJoy.getY() * RobotInformation.DriveTeamInfo.driverPercentage), ( -Robot.rightJoy.getY()* RobotInformation.DriveTeamInfo.driverPercentage));
            } //TODO: Adjust if sides are inverted
        }

        // Use negative number to move back and positive number to move forward
        //TODO: adjust kP to drive nice and smooth- if it takes to long just use the traditional moveBack and pray.java drivetrain is even
        public static void driveStraight(double power) {
            double kP = 0.0027;
            double correction = -Robot.gyro.getAngle();
            double turnPower = kP*correction;
            Robot.drive.arcadeDrive(power, turnPower, false);
        }

        //TODO: After testing, if the above function doesn't work, just switch to the one below this comment.
        public static void moveBack() {
            Robot.drive.tankDrive(-0.1, -0.1);
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
