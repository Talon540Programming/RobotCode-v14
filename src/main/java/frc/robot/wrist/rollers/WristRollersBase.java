package frc.robot.wrist.rollers;

import org.talon540.math.mapping.data.BoundDataset;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANDeviceIDS;

public class WristRollersBase extends SubsystemBase {
    private WPI_TalonSRX wristRollers;

    public BoundDataset resistanceMap = new BoundDataset(150);

    @Override
    public void periodic() {
        addDataPoint(wristRollers.getSupplyCurrent());
    }

    public boolean peakFound() {
        return ((7 * resistanceMap.getStandardDeviation()) < wristRollers.getSupplyCurrent() && 100 <= resistanceMap.getDatasetSize());
    }

    public WristRollersBase() {
        this.wristRollers = new WPI_TalonSRX(CANDeviceIDS.INTAKE_ROLLERS);
    }

    public void setRollers(double percentOutput) {
        this.wristRollers.set(ControlMode.PercentOutput, percentOutput);
    }

    public void stopRollers() {
        this.wristRollers.stopMotor();
    }

    public void addDataPoint(double point) {
        resistanceMap.addNode(point);
    }

    public void clearDataMap() {
        resistanceMap.clearDataset();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("Peak Found", this::peakFound, null);
    }
}
