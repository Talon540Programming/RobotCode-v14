package frc.robot.Modules.Mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.Robot;
import frc.robot.Modules.MotorControl;
import frc.robot.Modules.RobotInformation;

public class Intake {
    /** The spinny part that is duct taped together by "The Contractor"
         * If the right bumper is pressed, the intake spins from the field to the robot. If the left bumper is
         * pressed, the intake spins from the entrance of the robot to the flywheel. If neither bumper is
         * pressed, the intake is stopped
     */
    public static void rollers() { // Had absolute value of getRightY, would never be below zero!
        if (Robot.controller.getLeftBumper()) { //Right trigger spins intake from field to robot
            Robot.rollers.set(ControlMode.PercentOutput, -RobotInformation.DriveTeamInfo.rollerLowPercentage);
        } else if(Robot.controller.getAButton()) { //Left trigger spins intake from entrance of robot to flywheel
            Robot.rollers.set(ControlMode.PercentOutput, -RobotInformation.DriveTeamInfo.rollerHighPercentage);
        } else if(Robot.controller.getBButton()) { //Left trigger spins intake from flywheel to entrance of robot
            Robot.rollers.set(ControlMode.PercentOutput, RobotInformation.DriveTeamInfo.rollerLowPercentage);
        } else {
            Robot.rollers.set(ControlMode.PercentOutput, 0);
        }
    }

    /** The stupid thing that looks cool but doesnt really work // edit: it works apparently
     * This function is called when the left joystick is moved in the Y direction.
     * If the joystick is moved up, the wrist is set to the output of the joystick multiplied by the
     * transfer percentage.
     * If the joystick is moved down, the wrist is set to 0
     */
    public static void runWrist() {
        if(Math.abs(Robot.controller.getLeftY()) > 0.2) {
            Robot.wrist.set(ControlMode.PercentOutput, (Robot.controller.getLeftY() * -RobotInformation.DriveTeamInfo.wristTransferPercentage));
        } else {
            Robot.wrist.set(ControlMode.PercentOutput, 0);
        }
      }

    public static Boolean wristInside() {
        double wrist_position = MotorControl.getCurrentPosition(Robot.wrist);
        if(0 < wrist_position && wrist_position < 1) { //TODO: find wrist sensor positions that constitute the wrist is inside
            return true;
        } else {
            return false;
        }
    }

    public static Boolean wristDown() {
        double wrist_position = MotorControl.getCurrentPosition(Robot.wrist);
        if(0 < wrist_position && wrist_position < 1) { //TODO: find wrist sensor positions that constitute the wrist is down
            return true;
        } else {
            return false;
        }
    }

}
