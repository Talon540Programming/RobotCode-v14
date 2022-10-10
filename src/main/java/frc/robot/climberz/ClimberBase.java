package frc.robot.climberz;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class ClimberBase extends SubsystemBase {
    private WPI_TalonFX climbExtension, climbRotation;

    public ClimberBase() {
        this.climbExtension = new WPI_TalonFX(Constants.RobotData.RobotPorts.CLIMBEXTENSION);
        this.climbRotation = new WPI_TalonFX(Constants.RobotData.RobotPorts.CLIMBROTATION);

        this.climbExtension.setNeutralMode(NeutralMode.Brake);
        this.climbRotation.setNeutralMode(NeutralMode.Brake);
    }

    /**
     * Uniformally control both climber's extensions with the same output percent
     * @param percent percent output between [-1,1]
     */
    public void setExtensionPercentOut(double percent) {
        climbExtension.set(ControlMode.PercentOutput, percent);
    }
    
    /**
     * Control output to the climb rotation motor by a percent output
     * @param percent percent output between [-1,1]
     * 
     * @deprecated DO NOT USE, THIS CLIMBERZ SUCK BALLS AND THIS WILL BREAK THEM
     */
    public void setRotationPercentOut(double percent) {
        climbRotation.set(ControlMode.PercentOutput, percent);
    }

    /**
     * Stop the climb extension motor in place 
     * @implNote there is no pid control keeping climber in place so it solely relies on brake mode
     */
    public void brakeExtension() {
        climbExtension.stopMotor();
    }

    /**
     * Stop the climb rotation motor in place 
     * @implNote there is no pid control keeping climber in place so it solely relies on brake mode
     */
    public void brakeRotation() {
        climbRotation.stopMotor();
    }

}
