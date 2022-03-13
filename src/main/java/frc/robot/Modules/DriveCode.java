package frc.robot.Modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.Robot;

public class DriveCode {
    /** Main Drive Call */
    public static void tankDrive() {
        if ((Math.abs(Robot.rightJoy.getY()) > 0.2) || Math.abs(Robot.leftJoy.getY()) > 0.2) {
            Robot.drive.tankDrive((-Robot.rightJoy.getY() * RobotInformation.DriveTeamInfo.driverPercentage), (Robot.leftJoy.getY() * RobotInformation.DriveTeamInfo.driverPercentage));// TODO: adjust deadzones
        }
    }
    /** Move the Robot Backwards at -0.1 speed (should be a PID loop imo) */
    public static void moveBack() {
        Robot.drive.tankDrive(-0.1, -0.1);
    }

/**
 * Drive the robot using the left and right master motors at the given motor speeds.*
 * 
 * @param motorSpeedLeft The speed that you want the left motor to go.
 * @param motorSpeedRight The speed that you want the right side to go.
 */
    public static void oldDriveTrain(double motorSpeedLeft, double motorSpeedRight) {
        Robot.leftMaster.set(ControlMode.PercentOutput, motorSpeedLeft);
        Robot.rightMaster.set(ControlMode.PercentOutput, motorSpeedRight);
    }
}
