package frc.robot.drivetrain;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

// import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainBase extends SubsystemBase {
    private WPI_TalonFX rightLeader, rightFollower;
    private WPI_TalonFX leftLeader, leftFollower;

    private DifferentialDrive driveDifferential;
    // private DifferentialDriveKinematics driveKinematics = new
    // DifferentialDriveKinematics(Constants.RobotData.RobotMeasurement.botwidthMeters);

    public DrivetrainBase() {
        this.rightLeader = new WPI_TalonFX(Constants.RobotData.RobotPorts.DRIVETRAIN_FRONTRIGHT);
        this.rightFollower = new WPI_TalonFX(Constants.RobotData.RobotPorts.DRIVETRAIN_BACKRIGHT);

        this.leftLeader = new WPI_TalonFX(Constants.RobotData.RobotPorts.DRIVETRAIN_FRONTLEFT);
        this.leftFollower = new WPI_TalonFX(Constants.RobotData.RobotPorts.DRIVETRAIN_BACKLEFT);

        MotorControllerGroup rightDrive = new MotorControllerGroup(this.rightLeader, this.rightFollower);
        MotorControllerGroup leftDrive = new MotorControllerGroup(this.leftLeader, this.leftFollower);

        this.driveDifferential = new DifferentialDrive(leftDrive, rightDrive);

        /* It was CAD's fault */
        leftDrive.setInverted(true);
    }

    @Override
    public void periodic() {

    }

    /**
     * Independently control each side of the drivetrain with a specific speed
     * @param leftSpeed speed of drivetrain within [-1, 1]
     * @param rightSpeed speed of drivetrain within [-1, 1]
     */
    public void tankDrive(double leftSpeed, double rightSpeed) {
        driveDifferential.tankDrive(leftSpeed, rightSpeed);
    }

    /**
     * Stop the drivetrain motors in place
     */
    public void brake() {
        driveDifferential.stopMotor();
    }

}
