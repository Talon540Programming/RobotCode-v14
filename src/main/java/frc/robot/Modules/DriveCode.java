package frc.robot.Modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.Robot;

public class DriveCode {
    public static void tankDrive() {
        if ((Math.abs(Robot.rightJoy.getY()) > 0.2) || Math.abs(Robot.leftJoy.getY()) > 0.2) {
            Robot.drive.tankDrive((-Robot.rightJoy.getY() * RobotInformation.DriveTeamInfo.driverPercentage), (Robot.leftJoy.getY() * RobotInformation.DriveTeamInfo.driverPercentage));// TODO: adjust deadzones
        }
    }
    public static void moveBack() {
        Robot.drive.tankDrive(-0.1, -0.1);
    }
}
