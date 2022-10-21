package frc.robot.wrist.rotation;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANDeviceIDS;

public class WristRotationBase extends SubsystemBase {
    private WPI_TalonFX wristRotation;

    public final double resistanceTollerance = 4.8;

    public WristRotationBase() {
        this.wristRotation = new WPI_TalonFX(CANDeviceIDS.INTAKE_WRIST);
    }

    /**
     * Get resistance from the motor in amps.
     * As a motor induces a higher load, it will draw more amperage. We can use this
     * to our advantage by measuring amperage draw and using the last entered motor
     * input, to determine if the wrist is going up or down / if it is all the way
     * up or down. We can combine this with integrated encoder values to determine
     * with some degree of accuracy of the wrist's position.
     * 
     * @return resistance in amps
     */
    public double getResistance() {
        return wristRotation.getSupplyCurrent();
    }

    /**
     * Return if the wrist is under a significant load (such as being at the limit of its rotation)
     * @return
     */
    public boolean underResistance() {
        return getResistance() < resistanceTollerance;
    }

    /**
     * Set the wrist output percent.
     * Automatically stops the wrist if resistance is met over threshold:
     * {@link WristRotationBase#resistanceTollerance}
     * 
     * @param percentOutput Negetive value moves the wrist outside and positive
     *                      values move the wrist inside
     */
    public void setWrist(double percentOutput) {
        if (underResistance()) {
            this.wristRotation.set(ControlMode.PercentOutput, percentOutput);
        } else {
            // wristRotation.set(ControlMode.PercentOutput, 0);
            stopWrist();
        }
    }

    /**
     * Stop the wrist motor, will not run again till another input is given
     */
    public void stopWrist() {
        wristRotation.stopMotor();
    }
}
