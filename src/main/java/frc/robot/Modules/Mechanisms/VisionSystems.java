package frc.robot.Modules.Mechanisms;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Modules.RobotInformation;
import frc.robot.Modules.RobotInformation.FieldData;
import frc.robot.Modules.RobotInformation.RobotData;
import frc.robot.Modules.RobotInformation.FieldData.ValidTargets;
import frc.robot.Modules.RobotInformation.RobotData.RobotMeasurement;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Modules.Calculations;
import frc.robot.Modules.GameControl;

public class VisionSystems {
    /** Limelight Visions System. Used for Tracking Retro-Reflectors */
    public static class Limelight {
        /** The last angle seen by the limelight that wasn't zero */
        public static double nonZeroLimelightAngle; // Used to orient bot

        // This is a enum that is used to set the LEDS of the limelight.
        public static enum Limelight_Light_States {
            on,
            off,
            blink
        }

    /**
     * This function is used to update the SmartDashboard with the current values of the Limelight
     */
        public static void updateSmartDashboard() {
            NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

            double horizontalAngle = table.getEntry("tx").getDouble(0);
            double verticalAngle = table.getEntry("ty").getDouble(0);

            if(horizontalAngle != 0) {
                nonZeroLimelightAngle = horizontalAngle;
              }

            SmartDashboard.putNumber("Distance from Hubs",getDistanceFromHubStack());
            SmartDashboard.putNumber("Limelight H-Angle",horizontalAngle);
            SmartDashboard.putNumber("Limelight V-Angle",verticalAngle);
            SmartDashboard.putNumber("Non Zero Angle", nonZeroLimelightAngle);

            SmartDashboard.putNumber("Lower Hub Ball Velocity", Calculations.getDesiredBallVelocity(ValidTargets.lower_hub));
            SmartDashboard.putNumber("Upper hub Ball Velocity", Calculations.getDesiredBallVelocity(ValidTargets.upper_hub));

        }

    /**
     * This function returns the distance from the center of the limelight to the center of the hub stack
     *
     * @return The distance from the center of the limelight to the center of the hub stack.
     */
        public static double getDistanceFromHubStack() {
            NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
            double verticalAngle = table.getEntry("ty").getDouble(0);
            return (RobotInformation.FieldData.upperHubHeightMeters-RobotInformation.RobotData.RobotMeasurement.LimelightHeightMeters) / (Math.tan((RobotInformation.RobotData.RobotMeasurement.LimelightAngleRadians + Math.toRadians(verticalAngle))));
        }
    
        /**
     * Is the top hub present in the FOV?
     *
     * @return A boolean value.
     */
        public static boolean hubPresent() {
            NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
            double targetPresence = table.getEntry("tv").getDouble(0);
            if(targetPresence == 1) {
                return true;
            } else {
                return false;
            }
        }

    /**
     * Initialises the Limelight for Top Hub tracking: Sets the LEDS to on, sets the pipeline to 0, and sets the Limelight as a Vision Processor
     */
        public static void init() {
            setLEDS(Limelight_Light_States.blink);
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0); //Sets the pipeline to 0
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0); //Sets the Limelight as a Vision Proccesor
        }

    /**
     * Sets the pipeline to the value of pipeline
     *
     * @param pipeline 0-10
     */
        public static void setPipeline(double pipeline) {
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline); //Sets the pipeline to pipeline
        }

    /**
     * Code run for limelight when robot is in a disabled state
     */
        public static void disabled() {
            setLEDS(Limelight_Light_States.off); // Turns off the god damm limelight cause im going to go blind and gouge my eyes out because wtf does it need to be so bright like holy hell
        }


    /**
     * This function sets the LED mode of the limelight
     *
     * @param mode Limelight_Light_States on, off, blink
     */
        public static void setLEDS(Limelight_Light_States mode) {
            switch (mode) {
                case on:
                    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3); // light on
                    break;

                case off:
                    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1); //light off
                    break;

                case blink:
                    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(2); //light blinking
                    break;

                default:
                    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0); // as per pipeline mode (usually on)

            }
        }

    }

    /** Balltracking Vision Elements, used for tracking balls durring Auto */
    public static class BallTracking {
        public static double nonZeroBallAngle;

        /**
         * This function is called when auto first started.
         *
         * It sets the alliance color to the value of the alliance color selected in the SmartDashboard or from the FMS if none are selected
         */
        public static void updateCoprocessorValues() {
            NetworkTableInstance.getDefault().getTable("TalonPi").getEntry("Alliance Color").setString(GameControl.getAllianceColor());
            NetworkTableInstance.getDefault().getTable("TalonPi").getEntry("Gamemode").setString(GameControl.getCurrentGamemode());


        }
        
        public static void coprocessorErrorCheck() {
            String output_alliance = NetworkTableInstance.getDefault().getTable("TalonPi").getEntry("Alliance Color").getString("Not Ready");
            String input_alliance = NetworkTableInstance.getDefault().getTable("TalonPi").getEntry("Working Color").getString("Not Ready");
            if((output_alliance != input_alliance) && (output_alliance != "Not Ready")) {
                DriverStation.reportWarning("Raspberry Pi Alliance Mismatch. Currently Reporting: "+output_alliance+" Currently Reciving: "+input_alliance,false);
            }
        }
    }
}