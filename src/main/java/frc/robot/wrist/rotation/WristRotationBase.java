package frc.robot.wrist.rotation;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANDeviceIDS;

public class WristRotationBase extends SubsystemBase {
    private WPI_TalonFX wristRotation;

    private int lastInput = 0;
    private boolean wristDisabled = false;

    public WristRotationBase() {
        this.wristRotation = new WPI_TalonFX(CANDeviceIDS.INTAKE_WRIST);
    }

    @Override
    public void periodic() {
        double resistance = getResistance();

        if(resistance < 4.8) {
            wristDisabled = false;
        } else {
            wristDisabled = true;
            wristRotation.set(ControlMode.PercentOutput, 0);
        }
    }

    /**
     * Return the resistance faced by the wrist motor,
     * negetive value means the wrist is down, positive value means the wrist is
     * inside the robot, if the value is non-zero,
     * there is significant resistance
     */
    private double checkResistance() {
        double resistance = getResistance();

        if (lastInput == 0 || resistance < 5.1)
            return 0;

        return lastInput;

    }

    private double getResistance() {
        return wristRotation.getSupplyCurrent();
    }

    public void setWrist(double percentOutput) {
        if(wristDisabled) return;
        lastInput = (int) Math.copySign(1, percentOutput);
        this.wristRotation.set(ControlMode.PercentOutput, percentOutput);
    }

    public void stopWrist() {
        lastInput = 0;
        wristRotation.stopMotor();
    }
}
