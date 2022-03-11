package frc.robot.Modules;

import frc.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DigitalInput;



public class MotorContol {
    public static double getRPM(double idealVelocity, double transferPercent) {
        return (((idealVelocity*(1/transferPercent))/(Math.PI*0.1016))*60); // rudimentary calculation that's 90% wrong
    }

    public static double getCurrentVelocity(WPI_TalonFX motor) { // If using a TalonFX motor only
        return (motor.getSensorCollection().getIntegratedSensorVelocity());
    }

    public static double getCurrentPosition(WPI_TalonFX motor) { // If using a TalonFX motor only
        return (motor.getSensorCollection().getIntegratedSensorPosition());
    }
    
}
