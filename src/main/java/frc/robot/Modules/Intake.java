package frc.robot.Modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake {
    /** The spinny part that is duct taped together by "The Contractor" */
    public static void rollers() { // Had absolute value of getRightY, would never be below zero!
        if (Robot.controller.getLeftBumper()) { //Right trigger spins intake from field to robot
            Robot.rollers.set(ControlMode.PercentOutput, -RobotInformation.DriveTeamInfo.rollerLBTransferPercentage);
        } else if(Robot.controller.getAButton()) { //Left trigger spins intake from entrance of robot to flywheel
            Robot.rollers.set(ControlMode.PercentOutput, -RobotInformation.DriveTeamInfo.rollerBTransferPercentage);
        } else {
            Robot.rollers.set(ControlMode.PercentOutput, 0);
        }
    }
   
    /** The stupid thing that looks cool but doesnt really work // edit: it works apparently */
    public static void wrist() {
        if(Math.abs(Robot.controller.getLeftY()) > 0.2) {
            Robot.wrist.set(ControlMode.PercentOutput, (Robot.controller.getLeftY() * -RobotInformation.DriveTeamInfo.wristTransferPercentage));
        } else {
            Robot.wrist.set(ControlMode.PercentOutput, 0);
        }

        // if (Robot.controller.getBButton()) { //sets intake in position to feed ball into flywheel
        //   Robot.wrist.set(ControlMode.PercentOutput, 0.2);
        // } else if (Robot.controller.getXButton()) { //sets intake in position to pick balls off field
        //     Robot.wrist.set(ControlMode.PercentOutput, -0.2);
        // } else {
        //     Robot.wrist.set(ControlMode.PercentOutput, 0);
        // }
      }


}
