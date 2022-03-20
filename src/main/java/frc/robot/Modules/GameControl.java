package frc.robot.Modules;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class GameControl {
//TODO: Use the Display I (Aryan) made and the file I sent on Discord for Shuffleboard/SmartDashboard so that it looks pretty


    public static enum MatchTypes {
        auto,
        teleop_drive,
        teleop_climb,
        disabled
    }
    public static MatchTypes currentMatchType;

    public static enum ControllerStates {
        drive_mode,
        climb_mode
    }
    public static ControllerStates currentControllerState;

    
    public static SendableChooser<String> alliance_chooser = new SendableChooser<>();
    
    /**
     * This function is called to initalise Alliance color and ball tracking
     */
    public static void initializeAllianceChooser() {
        alliance_chooser.setDefaultOption("Select Alliance Color","N/A"); // Use FMS
        alliance_chooser.addOption("Red", "red");
        alliance_chooser.addOption("Blue", "blue");
        SmartDashboard.putData("Alliance Color for Ball Tracking", alliance_chooser);
    }

    /**
     * This function returns the alliance color of the robot
     *
     * @return The alliance color.
     */
    public static String getAllianceColor() {
        switch(DriverStation.getAlliance()) {
            case Blue: // If the enum is blue then go blue
                return "blue";

            case Red: // If the enum is red then go red
                return "red";

            default: // If the enum is invalid due to FMS error or non entry then return the sendable chooser option (better hope it isnt N/A lmao)
                return alliance_chooser.getSelected();
        }
    }

    public static String getCurrentGamemode() {
        switch(currentMatchType) {
            case auto:
                return "auto";
            case teleop_drive:
                return "teleop_drive";
            case teleop_climb:
                return "teleop_climb";
            case disabled:
                return "disabled";
            default:
                return null;
        }
    }

    public static class UserControl { // User interface
        public static RobotLEDState currentRobotLEDState;
        // public static Spark ledController = new Spark(0);

        public static enum rumbleSides {
            left,
            right,
            both
        }

        public static enum RobotLEDState {
            on,
            off,
            blink
        }

        /**
         * Set the rumble of the controller to a given value
         * 
         * @param side The side of the controller to rumble.
         * @param value Value to rumble controler at [0-1]
         */
        public static void setControllerRumble(rumbleSides side, double value) {
            switch(side) {
                case left:
                    Robot.controller.setRumble(RumbleType.kLeftRumble, value);
                    break;

                case right:
                    Robot.controller.setRumble(RumbleType.kRightRumble, value);
                    break;
                
                case both:
                    Robot.controller.setRumble(RumbleType.kRightRumble, value);
                    Robot.controller.setRumble(RumbleType.kLeftRumble, value);
                    break;
            }
        }
        

        public static void setRobotLEDS(RobotLEDState ledState) {
            switch(ledState) {
                case on:
                    // Turn LEDS on
                    SmartDashboard.putString("CURRENT LED STATE", "on");
                    // ledController.setVoltage(0);
                    currentRobotLEDState = RobotLEDState.on;
                    break;

                case off:
                    // Turn LEDS off
                    SmartDashboard.putString("CURRENT LED STATE", "off");
                    currentRobotLEDState = RobotLEDState.off;
                    break;

                case blink:
                    // Blink LEDS
                    SmartDashboard.putString("CURRENT LED STATE", "off");
                    currentRobotLEDState = RobotLEDState.blink;
                    break;
            }
        }
        public static class UserInterfaceControl {
            /**
             * This class is used to configure the Shuffleboard
             */
            public static class ShuffleboardConfig {
                
            }
            
            /**
             * This class is used to configure the SmartDashboard
             */
            public static class SmartDashboardConfig {

            }
        }
        
    }
}
