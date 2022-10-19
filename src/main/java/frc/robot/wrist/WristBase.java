package frc.robot.wrist;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANDeviceIDS;

public class WristBase extends SubsystemBase {
    private WPI_TalonFX wristRotation;
    private WPI_TalonSRX wristRollers;

    public WristBase() {
        this.wristRotation = new WPI_TalonFX(CANDeviceIDS.INTAKE_WRIST);
        this.wristRollers = new WPI_TalonSRX(CANDeviceIDS.INTAKE_ROLLERS);
    }

    public void setWrist(double percentOutput) {
        this.wristRotation.set(ControlMode.PercentOutput, percentOutput);
    }

    public void setRollers(double percentOutput) {
        percentOutput = MathUtil.clamp(percentOutput, -1, 1);
        this.wristRollers.set(ControlMode.PercentOutput, percentOutput);
    }

    public void stopRollers() {
        wristRollers.stopMotor();
    }

    public void stopWrist() {
        wristRotation.stopMotor();
    }
}
