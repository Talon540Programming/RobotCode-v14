package frc.robot.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

import org.talon540.math.TalonFXIntegratedSensorManager;
import org.talon540.math.conversions;

public class ShooterBase extends SubsystemBase {
    private WPI_TalonFX flywheelMotor;
    private TalonFXIntegratedSensorManager sensorCollection;

    public ShooterBase() {
        this.flywheelMotor = new WPI_TalonFX(Constants.RobotData.RobotPorts.SHOOTER_FLY);
        this.sensorCollection = new TalonFXIntegratedSensorManager(
            this.flywheelMotor.getSensorCollection(), 
            Units.inchesToMeters(Constants.RobotData.RobotMeasurement.WheelData.Flywheel.FlywheelDiameter / 2), 
            Constants.RobotData.MotorData.Shooter.Flywheel.gearRatio
        );
    }

    /** Return the RPM of the flywheel */
    public double getFlywheelRPM() {
        return this.sensorCollection.getObjectRPM();
    }

    /**
     * @return Angular velocity of the flywheel in meters per second
     */
    public double getFlywheelLinearVelocity() {
        return this.sensorCollection.getLinearVelocity();
    }

    /**
     * @return Linear velocity of the flywheel in meters per second
     */
    public double getFlywheelAngularVelocity() {
        return this.sensorCollection.getAngularVelocity();
    }

    /**
     * Set the {@code FLYWHEEL} RPM
     * @param targetRPM RPM to set {@code FLYWHEEL} to
     */
    public void setFlywheelRPM(double targetRPM) {
        targetRPM = MathUtil.clamp(targetRPM, -6380, 6380);

        this.flywheelMotor.set(ControlMode.Velocity, conversions.RPMtoFalcon500Velocity(targetRPM) * Constants.RobotData.MotorData.Shooter.Flywheel.gearRatio);
    }

    public void setFlywheelLinearVelocity(double targetVelocity) {
        this.setFlywheelRPM(conversions.LinearVelocityToRPM(targetVelocity, Units.inchesToMeters(Constants.RobotData.RobotMeasurement.WheelData.Flywheel.FlywheelDiameter / 2)));
    }

    /**
     * Set {@code FLYWHEEL} RPM to certain angular velocity
     * @param targetVelocity angular velocity to set {@code FLYWHEEL} to
     */
    public void setFlywheelAngularVelocity(double targetVelocity) {
        this.setFlywheelRPM(conversions.AngularVelocityToRPM(targetVelocity));
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
        builder.addDoubleProperty("Flywheel Linear Velocity", this::getFlywheelLinearVelocity, this::setFlywheelLinearVelocity);
        builder.addDoubleProperty("Flywheel Angular Velocity", this::getFlywheelAngularVelocity, this::setFlywheelAngularVelocity);

    }
}
