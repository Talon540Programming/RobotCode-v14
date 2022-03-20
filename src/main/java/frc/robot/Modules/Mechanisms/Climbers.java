package frc.robot.Modules.Mechanisms;

import frc.robot.Robot;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class Climbers {
    public static enum climbers {
        extension,
        rotation
    }
    public static enum rotationPositions {
        // Climb Rotation
        idle,
        extensionReady,

        // Climb Extension
        // TBD
    }

    public static void setPosition(climbers climber, rotationPositions position) {
        switch(climber) {
            case extension:

                break;
            case rotation:
                // double currentRotationPosition = gyro
                switch(position) {
                    case idle:
                        
                        break;
                    case extensionReady:

                        break;

                    default:
                }
                break;
        }
    }





    //Vertical Climb, no encoder
/**
 * If the D pad is up, then the climb extension motor will be set to 1. If the D pad is down, then the
 * climb extension motor will be set to -1. Otherwise, the climb extension motor will be set to 0
 */
    public static void climb() {
        if (Robot.controller.getPOV() == 0) { //Up on D pad
            Robot.climbExtension.set(ControlMode.PercentOutput, 0.6);
        } else if (Robot.controller.getPOV() == 180) { //Down on D pad
            Robot.climbExtension.set(ControlMode.PercentOutput, -0.6);
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