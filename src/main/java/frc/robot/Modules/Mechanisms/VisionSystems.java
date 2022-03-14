package frc.robot.Modules.Mechanisms;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Modules.RobotInformation;
import frc.robot.Modules.RobotInformation.FieldData;
import frc.robot.Modules.RobotInformation.RobotData;
import frc.robot.Modules.RobotInformation.FieldData.ValidTargets;
import frc.robot.Modules.RobotInformation.RobotData.RobotMeasurement;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        public static void updateAllianceColor() {
            NetworkTableInstance.getDefault().getTable("TalonPi").getEntry("Alliance Color").setString(GameControl.getAllianceColor());
        }

    }

    /** Calculations pertaining to vision systems */
    public static class Calculations {

        /**
         * This function calculates the desired velocity of the flywheel for a given target
         *
         * @param target The target that the robot is shooting at.
         * @return The desired velocity of the ball.
         */
        public static double getDesiredBallVelocity(ValidTargets target) {
            switch(target) {
                case upper_hub:
                    return Math.sqrt(-1 * ((9.8 * VisionSystems.Limelight.getDistanceFromHubStack() * VisionSystems.Limelight.getDistanceFromHubStack() * (1 + (Math.pow(Math.tan(Math.toRadians(RobotInformation.RobotData.RobotMeasurement.shooterHoodAngle)), 2))) )/((2 * (RobotInformation.FieldData.upperHubHeightMeters-RobotInformation.RobotData.RobotMeasurement.flywheelHeightMeters))-(2 * VisionSystems.Limelight.getDistanceFromHubStack() * Math.tan(Math.toRadians(RobotInformation.RobotData.RobotMeasurement.shooterHoodAngle))))));
                case lower_hub:
                    return Math.sqrt(-1 * ((9.8 * VisionSystems.Limelight.getDistanceFromHubStack() * VisionSystems.Limelight.getDistanceFromHubStack() * (1 + (Math.pow(Math.tan(Math.toRadians(RobotInformation.RobotData.RobotMeasurement.shooterHoodAngle)), 2))) )/((2 * (RobotInformation.FieldData.lowerHubHeightMeters-RobotInformation.RobotData.RobotMeasurement.flywheelHeightMeters))-(2 * VisionSystems.Limelight.getDistanceFromHubStack() * Math.tan(Math.toRadians(RobotInformation.RobotData.RobotMeasurement.shooterHoodAngle))))));
                default:
                    return 0;
            }
        }
    }
}