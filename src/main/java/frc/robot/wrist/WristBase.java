package frc.robot.wrist;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANDeviceIDS;
import frc.robot.constants.Constants;

public class WristBase extends SubsystemBase {
    private WPI_TalonFX wristRotation, wristRollers;

    public WristBase() {
        this.wristRotation = new WPI_TalonFX(CANDeviceIDS.INTAKE_WRIST);
        this.wristRollers = new WPI_TalonFX(CANDeviceIDS.INTAKE_ROLLERS);
    }

    public void setWrist(double percentOutput) {
        percentOutput *= Constants.wristTransferPercentage;

        this.wristRotation.set(ControlMode.PercentOutput, percentOutput);
    }

    public void setRollers(double percentOutput) {
        percentOutput = MathUtil.clamp(percentOutput, -1, 1);
        if(percentOutput < 0) {
            percentOutput *= Constants.rollerHighPercentage;
        } else {
            percentOutput *=Constants.rollerLowPercentage;
        }

        this.wristRollers.set(ControlMode.PercentOutput, percentOutput);
    }

    public void stopRollers() {
        wristRollers.stopMotor();
    }

    public void stopWrist() {
        wristRotation.stopMotor();
    }
}
