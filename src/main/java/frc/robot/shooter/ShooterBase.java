package frc.robot.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterBase extends SubsystemBase {
    private WPI_TalonFX flywheelMotor;
    private TalonFXSensorCollection flywheelIntegratedEncoder;

    public ShooterBase() {
        this.flywheelMotor = new WPI_TalonFX(Constants.RobotData.RobotPorts.SHOOTER_FLY);
        this.flywheelIntegratedEncoder = this.flywheelMotor.getSensorCollection();
    }

    public double getFlywheelRPM() {
        return this.flywheelIntegratedEncoder.getIntegratedSensorVelocity() * (2048/600) / Constants.RobotData.MotorData.Shooter.Flywheel.gearRatio;
    }

    /**
     * @return Angular velocity of the flywheel in meters per second
     */
    public double getFlywheelVelocity() {
        return this.getFlywheelRPM() / 60 * Units.inchesToMeters(Constants.RobotData.RobotMeasurement.WheelData.Flywheel.FlywheelDiameter / 2);
    }

    /**
     * Set the {@code FLYWHEEL} RPM
     * @param targetRPM RPM to set {@code FLYWHEEL} to
     */
    public void setFlywheelRPM(double targetRPM) {
        MathUtil.clamp(targetRPM, -6380, 6380);
        targetRPM *= (600/2048);
        this.flywheelMotor.set(ControlMode.Velocity, targetRPM);
    }

    /**
     * Set {@code FLYWHEEL} RPM to certain angular velocity
     * @param targetVelocity angular velocity to set {@code FLYWHEEL} to
     */
    public void setFlywheelVelocity(double targetVelocity) {
        double targetRPM = targetVelocity / Units.inchesToMeters(Constants.RobotData.RobotMeasurement.WheelData.Flywheel.FlywheelDiameter / 2) * 60;
        this.setFlywheelRPM(targetRPM);
    }

    /**
     * Set {@code FLYWHEEL} speed to percent
     * @param percent power in domain [-1,1]
     */
    public void setPercentOutput(double percent) {
        percent = MathUtil.clamp(percent, -1, 1);
        this.flywheelMotor.set(ControlMode.PercentOutput, percent);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("shooter");

        builder.addDoubleProperty("Flywheel RPM", this::getFlywheelRPM, this::setFlywheelRPM);
        builder.addDoubleProperty("Flywheel Velocity", this::getFlywheelVelocity, this::setFlywheelVelocity);

    }
}
