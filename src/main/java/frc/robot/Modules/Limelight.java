package frc.robot.Modules;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight {
    /** The last angle seen by the limelight that wasn't zero */
    public static double nonZeroLimelightHorAng; // Used to orient bot

    public static enum Limelight_Light_States {
        on,
        off,
        blink
    }

/**
 * This function is used to update the SmartDashboard with the current values of the Limelight
 */
    public static void updateSmartDashboard() {
        double[][] shooterCalculations = Limelight.getLimelightData();
        SmartDashboard.putNumber("Distance: ",shooterCalculations[0][0]);
        SmartDashboard.putNumber("Shooter Angle: ",shooterCalculations[0][1]);
        SmartDashboard.putNumber("Ideal Ball Velocity :",shooterCalculations[0][2]);
        SmartDashboard.putNumber("Limelight H-Angle: ",shooterCalculations[1][0]);
        SmartDashboard.putNumber("Limelight V-Angle: ",shooterCalculations[1][1]);
        SmartDashboard.putNumber("Limelight Latency: ",shooterCalculations[1][2]);
        SmartDashboard.putNumber("Non Zero Angle", nonZeroLimelightHorAng);
    }

/**
 * This function returns the distance between the limelight and the retroreflector, the optimal angle
 * to shoot at, and the optimal ball velocity
 * 
 * @return The horizontal angle, vertical angle, and latency of the limelight.
 *  <ul> 
        <li>LimelightInfo[0][0] = distance; //Horizontal distance between the hubs and the limelight
        <li>LimelightInfo[0][1] = angle; //Optimal Shooter Angle
        <li>LimelightInfo[0][2] = velocity; //Optimal Ball velocity
        <li>LimelightInfo[1][0] = horizontalAngle; //Horizontal angle between the limelight and the retroreflector
        <li>LimelightInfo[1][1] = verticalAngle; //Vertical angle between limelight and retroreflector
        <li>LimelightInfo[1][2] = limelightLatency; // Latency for limelight calculations
 *  </ul>
 */
    public static double[][] getLimelightData() {
        double[][] LimelightInfo = {{0,0,0},{0,0,0}};
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

        double horizontalAngle = table.getEntry("tx").getDouble(0);
        double verticalAngle = table.getEntry("ty").getDouble(0);
        double limelightLatency = table.getEntry("tl").getDouble(0);

        double distance = (RobotInformation.FieldData.upperHubHeightMeters-RobotInformation.RobotData.RobotMeasurement.LimelightHeightMeters) / (Math.tan((RobotInformation.RobotData.RobotMeasurement.LimelightAngleRadians + Math.toRadians(verticalAngle))));
        double angle = Math.toDegrees(Math.atan((Math.tan(Math.toRadians(-RobotInformation.RobotData.ShooterData.hubEntryAngle)) * (distance)-(2 * (RobotInformation.FieldData.upperHubHeightMeters-RobotInformation.RobotData.RobotMeasurement.LimelightHeightMeters))) / (-distance)));
        double velocity = Math.sqrt(-1 * ((9.8 * distance * distance * (1 + (Math.pow(Math.tan(Math.toRadians(angle)), 2))) )/((2 * (RobotInformation.FieldData.upperHubHeightMeters-RobotInformation.RobotData.RobotMeasurement.LimelightHeightMeters))-(2 * distance * Math.tan(Math.toRadians(angle))))));

        LimelightInfo[0][0] = distance; //Horizontal distance between the hubs and the limelight
        LimelightInfo[0][1] = angle; //Optimal Shooter Angle
        LimelightInfo[0][2] = velocity; //Optimal Ball velocity
        LimelightInfo[1][0] = horizontalAngle; //Horizontal angle between the limelight and the retroreflector
        LimelightInfo[1][1] = verticalAngle; //Vertical angle between limelight and retroreflector
        LimelightInfo[1][2] = limelightLatency; // Latency for limelight calculations

        if(horizontalAngle != 0) {
          nonZeroLimelightHorAng = horizontalAngle;
        }

        return LimelightInfo;
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
        setLEDS(Limelight_Light_States.on);
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
 * @param mode The LED mode. ("off", "blink", "on")
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
