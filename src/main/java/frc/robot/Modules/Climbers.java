package frc.robot.Modules;

import frc.robot.Robot;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class Climbers {

    //Vertical Climb, no encoder
/**
 * If the D pad is up, then the climb extension motor will be set to 1. If the D pad is down, then the
 * climb extension motor will be set to -1. Otherwise, the climb extension motor will be set to 0
 */
    public static void climb() {
        if (Robot.controller.getPOV() == 0) { //Up on D pad
            Robot.climbExtension.set(ControlMode.PercentOutput, 1);
        } else if (Robot.controller.getPOV() == 180) { //Down on D pad
            Robot.climbExtension.set(ControlMode.PercentOutput, -1);
        } else {
            Robot.climbExtension.set(ControlMode.PercentOutput, 0);
        }
    }

    //Climb Rotation, no encoder
/**
 * If the D pad is pointed right, rotate the climber clockwise. If the D pad is pointed left, rotate
 * the climber counterclockwise. Otherwise, stop the climber.
 */
    public static void climbrotation() {
        if (Robot.controller.getPOV() == 90) { // Right on the D pad
            Robot.climbRotation.set(ControlMode.PercentOutput, 1);
        } else if (Robot.controller.getPOV() == 270) { //Left on the D pad
            Robot.climbRotation.set(ControlMode.PercentOutput, -1);
        } else {
            Robot.climbRotation.set(ControlMode.PercentOutput, 0);
        }
    }
}
