package frc.robot.wrist.rollers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANDeviceIDS;

public class WristRollersBase extends SubsystemBase {
    private WPI_TalonSRX wristRollers;

    public WristRollersBase() {
        this.wristRollers = new WPI_TalonSRX(CANDeviceIDS.INTAKE_ROLLERS);
    }

    public void setRollers(double percentOutput) {
        this.wristRollers.set(ControlMode.PercentOutput, percentOutput);
    }

    public void stopRollers() {
        this.wristRollers.stopMotor();
    }
    
}
