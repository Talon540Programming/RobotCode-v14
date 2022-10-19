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
import frc.robot.constants.CANDeviceIDS;
import frc.robot.constants.Measurements;
import frc.robot.constants.PID;

import org.talon540.mapping.BoundRobotPositionTreeMap;

import com.kauailabs.navx.frc.AHRS;

public class DrivetrainBase extends SubsystemBase {
    private TalonFX_DifferentialMotorGroup leftSide, rightSide;

    private DifferentialDrive driveDifferential;

    private DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(Measurements.Robot.botwidthMeters);
    private DifferentialDriveOdometry driveOdometry;
    private BoundRobotPositionTreeMap positionMap = new BoundRobotPositionTreeMap(500);


    private PIDController leftDriveController = new PIDController(
            PID.drivetrain.translation.kP,
            PID.drivetrain.translation.kI,
            PID.drivetrain.translation.kD
    );
    private PIDController rightDriveController = new PIDController(
            PID.drivetrain.translation.kP,
            PID.drivetrain.translation.kI,
            PID.drivetrain.translation.kD
    );
    public ProfiledPIDController rotationController = new ProfiledPIDController(
            PID.drivetrain.rotation.kP,
            PID.drivetrain.rotation.kI,
            PID.drivetrain.rotation.kD,
            new TrapezoidProfile.Constraints(
                    Measurements.Calculations.kMaxDrivetrainRotationalVelocity,
                    Measurements.Calculations.kMaxDrivetrainRotationalAcceleration
            )
    );

    public AHRS gyro;

    public DrivetrainBase(AHRS gyro) {
        this.gyro = gyro;

        WPI_TalonFX rightLeader = new WPI_TalonFX(CANDeviceIDS.DRIVETRAIN_FRONTRIGHT);
        WPI_TalonFX rightFollower = new WPI_TalonFX(CANDeviceIDS.DRIVETRAIN_BACKRIGHT);

        WPI_TalonFX leftLeader = new WPI_TalonFX(CANDeviceIDS.DRIVETRAIN_FRONTLEFT);
        WPI_TalonFX leftFollower = new WPI_TalonFX(CANDeviceIDS.DRIVETRAIN_BACKLEFT);

        rightSide = new TalonFX_DifferentialMotorGroup(rightLeader, rightFollower);
        leftSide = new TalonFX_DifferentialMotorGroup(leftLeader, leftFollower);

        /* It was CAD's fault */
        leftSide.setInverted(true);

        this.driveDifferential = new DifferentialDrive(leftSide, rightSide);
        this.driveDifferential.setSafetyEnabled(false);

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
        Pose2d currentPose = driveOdometry.update(gyro.getRotation2d(), getLeftPosition(), getRightPosition());
        positionMap.addPositionToMap(currentPose, Timer.getFPGATimestamp());
    }

    /**
     * Rotate the front of the drivetrain to a specific angle [-π, π]
     * 
     * @param rotationAngRad
     */
    public void rotateToAngle(double rotationAngRad) {
        double calc = rotationController.calculate(gyro.getRotation2d().getRadians(), rotationAngRad);
        calc = MathUtil.clamp(calc, -1, 1);

        driveDifferential.arcadeDrive(0, calc);
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
     * Equivalent to {@link DrivetrainBase#driveDifferential} tank drive without applying deadband values.
     * @implNote REMOVES ALL DEADBAND VALUES, SPEED LIMITATIONS, ETC.
     * @param leftPercent  speed of drivetrain within [-1, 1]
     * @param rightPercent speed of drivetrain within [-1, 1]
     */
    public void percentTankDrive(double leftPercent, double rightPercent) {
        this.leftSide.set(leftPercent);
        this.rightSide.set(rightPercent);
    }

    /**
     * Set drivetrain speed based on linear velocities of each side of the
     * drivetrain
     * 
     * @param leftSpeed  left side velocity in meters per second
     * @param rightSpeed right side velocity in meters per second
     */
    public void driveFromSpeeds(DifferentialDriveWheelSpeeds speeds) {
        double leftVal = 0;
        double rightVal = 0;

        if (speeds.leftMetersPerSecond > 2E-2)
            leftVal = MathUtil.clamp(leftDriveController.calculate(getLeftVelocity(), speeds.leftMetersPerSecond), -1, 1);

        if (speeds.rightMetersPerSecond > 2E-2)
            rightVal = MathUtil.clamp(rightDriveController.calculate(getRightVelocity(), speeds.rightMetersPerSecond), -1, 1);

        driveDifferential.tankDrive(leftVal, rightVal);
    }

    /**
     * Set the drivetrain speed from a {@link ChassisSpeeds} object
     * 
     * @param speed ChassisSpeeds to set drivetrain to
     */
    public void driveFromChassisSpeeds(ChassisSpeeds speed) {
        driveFromSpeeds(driveKinematics.toWheelSpeeds(speed));
    }

    /**
     * Get velocity of the left side of the drivetrain from the leader motor
     * @return velocity in meters per second
     */
    public double getLeftVelocity() {
        return conversions.Falcon500VelocityToLinearVelocity(
            leftSide.getVelocity(),
            Measurements.Robot.drivetrainWheelRadiusMeters,
            Measurements.Robot.GearRatios.drivetrain
        );
    }

    /**
     * Get velocity of the right side of the drivetrain from the leader motor
     * @return velocity in meters per second
     */
    public double getRightVelocity() {
        return conversions.Falcon500VelocityToLinearVelocity(
            rightSide.getVelocity(),
            Measurements.Robot.drivetrainWheelRadiusMeters,
            Measurements.Robot.GearRatios.drivetrain
        );
    }

    /**
     * Return the position of the left side of the drivetrain from the leader motor
     * @return position in meters
     */
    public double getLeftPosition() {
        return (leftSide.getPosition() / Measurements.Robot.GearRatios.drivetrain) * ((2 * Math.PI * Measurements.Robot.drivetrainWheelRadiusMeters) / 2048.0);
    }

    /**
     * Return the position of the right side of the drivetrain from the leader motor
     * @return position in meters
     */
    public double getRightPosition() {
        return (rightSide.getPosition() / Measurements.Robot.GearRatios.drivetrain) * ((2 * Math.PI * Measurements.Robot.drivetrainWheelRadiusMeters) / 2048.0);
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
     * Drive straight in a linear direction
     * @param percent between [-1,1], positive is forward
     */
    public void driveStraight(double percent) {

    }

    /**
     * Drive the robot from speeds
     * @param speedX Linear speed in {@code m/s}
     * @param speedRot Rotational speed in {@code rad/s}
     */
    public void driveFromForces(double speedX, double speedRot) {
        driveFromChassisSpeeds(new ChassisSpeeds(speedX, 0.0, speedRot));
    }

    public void resetGyro() {
        this.gyro.reset();
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
