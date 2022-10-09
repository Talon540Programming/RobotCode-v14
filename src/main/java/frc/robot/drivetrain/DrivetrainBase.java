package frc.robot.drivetrain;

import org.talon540.TalonFX_DifferentialMotorGroup;
import org.talon540.math.conversions;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

import org.talon540.mapping.BoundRobotPositionTreeMap;

import com.kauailabs.navx.frc.AHRS;

public class DrivetrainBase extends SubsystemBase {
    private TalonFX_DifferentialMotorGroup leftSide, rightSide;

    private DifferentialDrive driveDifferential;

    private DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(Constants.RobotData.RobotMeasurement.botwidthMeters);
    private DifferentialDriveOdometry driveOdometry;
    private BoundRobotPositionTreeMap positionMap = new BoundRobotPositionTreeMap(500);


    private PIDController leftDriveController = new PIDController(
            Constants.PID_Values.drivetrain.translation.kP,
            Constants.PID_Values.drivetrain.translation.kI,
            Constants.PID_Values.drivetrain.translation.kD);
    private PIDController rightDriveController = new PIDController(
            Constants.PID_Values.drivetrain.translation.kP,
            Constants.PID_Values.drivetrain.translation.kI,
            Constants.PID_Values.drivetrain.translation.kD);
    private ProfiledPIDController rotationController = new ProfiledPIDController(
            Constants.PID_Values.drivetrain.rotation.kP,
            Constants.PID_Values.drivetrain.rotation.kI,
            Constants.PID_Values.drivetrain.rotation.kD,
            new TrapezoidProfile.Constraints(
                    Constants.kMaxDrivetrainRotationalVelocity,
                    Constants.kMaxDrivetrainRotationalAcceleration));

    private AHRS gyro;

    public DrivetrainBase(AHRS gyro) {
        this.gyro = gyro;

        WPI_TalonFX rightLeader = new WPI_TalonFX(Constants.RobotData.RobotPorts.DRIVETRAIN_FRONTRIGHT);
        WPI_TalonFX rightFollower = new WPI_TalonFX(Constants.RobotData.RobotPorts.DRIVETRAIN_BACKRIGHT);

        WPI_TalonFX leftLeader = new WPI_TalonFX(Constants.RobotData.RobotPorts.DRIVETRAIN_FRONTLEFT);
        WPI_TalonFX leftFollower = new WPI_TalonFX(Constants.RobotData.RobotPorts.DRIVETRAIN_BACKLEFT);

        rightSide = new TalonFX_DifferentialMotorGroup(rightLeader, rightFollower);
        leftSide = new TalonFX_DifferentialMotorGroup(leftLeader, leftFollower);

        this.driveDifferential = new DifferentialDrive(leftSide, rightSide);

        /* It was CAD's fault */
        leftSide.setInverted(true);

        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        driveOdometry = new DifferentialDriveOdometry(gyro.getRotation2d());
    }

    @Override
    public void periodic() {
        updateOdometry();
    } 

    /**
     * Update the position of the robot from current meausrements and log the latest position in the position map for future reference
     */
    private void updateOdometry() {
        driveOdometry.update(gyro.getRotation2d(), getLeftPosition(), getRightPosition());
        positionMap.addPositionToMap(driveOdometry.getPoseMeters(), Timer.getFPGATimestamp());
    }

    /**
     * Rotate the front of the drivetrain to a specific angle [-π, π]
     * 
     * @param rotationAngRad
     */
    public void rotateToAngle(double rotationAngRad) {
        double calc = rotationController.calculate(gyro.getRotation2d().getRadians(), rotationAngRad);

        if(rotationAngRad < 0) {
            driveDifferential.tankDrive(-calc, calc);
        } else if(0 < rotationAngRad) {
            driveDifferential.tankDrive(calc, -calc);
        }
    }

    /**
     * Independently control each side of the drivetrain with a specific speed
     * 
     * @param leftSpeed  speed of drivetrain within [-1, 1]
     * @param rightSpeed speed of drivetrain within [-1, 1]
     */
    public void tankDrive(double leftSpeed, double rightSpeed) {
        driveDifferential.tankDrive(leftSpeed, rightSpeed);
    }

    /**
     * Set drivetrain speed based on linear velocities of each side of the
     * drivetrain
     * 
     * @param leftSpeed  left side velocity in meters per second
     * @param rightSpeed right side velocity in meters per second
     */
    public void driveFromSpeeds(double leftSpeed, double rightSpeed) {
        double leftVal = 0;
        double rightVal = 0;

        if (leftSpeed > 2E-2)
            leftVal = MathUtil.clamp(leftDriveController.calculate(getLeftVelocity(), leftSpeed), -1, 1);

        if (rightSpeed > 2E-2)
            rightVal = MathUtil.clamp(rightDriveController.calculate(getRightVelocity(), rightSpeed), -1, 1);

        driveDifferential.tankDrive(leftVal, rightVal);
    }

    /**
     * Set the drivetrain speed from a {@link ChassisSpeeds} object
     * 
     * @param speed ChassisSpeeds to set drivetrain to
     */
    public void driveFromChassisSpeeds(ChassisSpeeds speed) {
        DifferentialDriveWheelSpeeds speeds = driveKinematics.toWheelSpeeds(speed);
        driveFromSpeeds(speeds.leftMetersPerSecond, speeds.rightMetersPerSecond);
    }

    /**
     * Get velocity of the left side of the drivetrain from the leader motor
     * @return velocity in meters per second
     */
    public double getLeftVelocity() {
        return conversions.Falcon500VelocityToLinearVelocity(
            leftSide.getVelocity(),
            Constants.RobotData.RobotMeasurement.WheelData.DriveTrain.wheelRadiusMeters,
            Constants.RobotData.MotorData.Drivetrain.gearRatio
        );
    }

    /**
     * Get velocity of the right side of the drivetrain from the leader motor
     * @return velocity in meters per second
     */
    public double getRightVelocity() {
        return conversions.Falcon500VelocityToLinearVelocity(
            rightSide.getVelocity(),
            Constants.RobotData.RobotMeasurement.WheelData.DriveTrain.wheelRadiusMeters,
            Constants.RobotData.MotorData.Drivetrain.gearRatio
        );
    }

    /**
     * Return the position of the left side of the drivetrain from the leader motor
     * @return position in meters
     */
    public double getLeftPosition() {
        return (leftSide.getPosition() / Constants.RobotData.MotorData.Drivetrain.gearRatio) * ((2 * Math.PI * Constants.RobotData.RobotMeasurement.WheelData.DriveTrain.wheelRadiusMeters) / 2048.0);
    }

    /**
     * Return the position of the right side of the drivetrain from the leader motor
     * @return position in meters
     */
    public double getRightPosition() {
        return (rightSide.getPosition() / Constants.RobotData.MotorData.Drivetrain.gearRatio) * ((2 * Math.PI * Constants.RobotData.RobotMeasurement.WheelData.DriveTrain.wheelRadiusMeters) / 2048.0);
    }

    /**
     * Return the drivetrain velocity
     * 
     * @return
     */
    public double getAverageVelocity() {
        double leftVelocity = getLeftVelocity();
        double rightVelocity = getRightVelocity();

        return (leftVelocity + rightVelocity) / 2.0;
    }

    /**
     * Stop the drivetrain motors in place
     */
    public void brake() {
        driveDifferential.stopMotor();
    }

    /**
     * @return Get the current position of the robot on the field
     */
    public Pose2d getPosition() {
        return this.driveOdometry.getPoseMeters();
    }

    /**
     * Get robot's position from the position map using a timestamp
     * @param timestamp
     * @return Robot's position from map
     */
    public Pose2d getPosition(double timestamp) {
        return this.positionMap.getPositionFromTimestamp(timestamp);
    }

}
