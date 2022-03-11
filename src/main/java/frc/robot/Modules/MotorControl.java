package frc.robot.Modules;

import frc.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DigitalInput;



public class MotorControl {
    public static double getRPM(double idealVelocity, double transferPercent) {
        return (((idealVelocity*(1/transferPercent))/(Math.PI*0.1016))*60); // rudimentary calculation that's 90% wrong
    }

    public static double getCurrentVelocity(WPI_TalonFX motor) { // If using a TalonFX motor only
        return (motor.getSensorCollection().getIntegratedSensorVelocity());
    }

    public static double getCurrentPosition(WPI_TalonFX motor) { // If using a TalonFX motor only
        return (motor.getSensorCollection().getIntegratedSensorPosition());
    }

    public static void flywheel() {
        if(Robot.controller.getRightBumper()) {
            Robot.shooterFly.set(ControlMode.PercentOutput, 1);
        } else {
            Robot.shooterFly.set(ControlMode.PercentOutput, 0);
        }
    }
    
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
        // config P,I,D,F values- start by doubling F, then P, then D, then I (middle values) then increase/decrease over time
        // Robot.shooterFly.config_kF(0, RobotInformation.PID_Values.flywheel.kF, 1); // (F) Feed Forward Term
        // Robot.shooterFly.config_kP(0, RobotInformation.PID_Values.flywheel.kP, 1); // (P) Proportional Term
        // Robot.shooterFly.config_kI(0, RobotInformation.PID_Values.flywheel.kI, 1); // (I) Integral term
        // Robot.shooterFly.config_kD(0, RobotInformation.PID_Values.flywheel.kD, 1); // (D) Differentiable Term
    
    }
}
