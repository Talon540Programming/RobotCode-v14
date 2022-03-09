package frc.robot.Modules;

import frc.robot.Robot;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class Climbers {
    //Vertical Climb, no encoder
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
