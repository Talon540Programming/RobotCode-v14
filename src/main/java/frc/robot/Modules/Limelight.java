package frc.robot.Modules;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight {
    public static double nonZeroLimelightHorAng; // Used to orient bot

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
    
    public static double[][] getLimelightData() {
        double[][] LimelightInfo = {{0,0,0},{0,0,0}};
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    
        double horizontalAngle = table.getEntry("tx").getDouble(0);
        double verticalAngle = table.getEntry("ty").getDouble(0);
        double limelightLatency = table.getEntry("tl").getDouble(0);
    
        double distance = (RobotInformation.FeildData.upperHubHeightMeters-RobotInformation.RobotData.RobotMeasurement.LimelightHeightMeters) / (Math.tan((RobotInformation.RobotData.RobotMeasurement.LimelightAngleRadians + Math.toRadians(verticalAngle))));
        double angle = Math.toDegrees(Math.atan((Math.tan(Math.toRadians(-RobotInformation.RobotData.ShooterData.hubEntryAngle)) * (distance)-(2 * (RobotInformation.FeildData.upperHubHeightMeters-RobotInformation.RobotData.RobotMeasurement.LimelightHeightMeters))) / (-distance)));
        double velocity = Math.sqrt(-1 * ((9.8 * distance * distance * (1 + (Math.pow(Math.tan(Math.toRadians(angle)), 2))) )/((2 * (RobotInformation.FeildData.upperHubHeightMeters-RobotInformation.RobotData.RobotMeasurement.LimelightHeightMeters))-(2 * distance * Math.tan(Math.toRadians(angle))))));
    
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

    public static boolean hubPresent() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        double targetPresence = table.getEntry("tv").getDouble(0);
        if(targetPresence == 1) {
            return true;
        } else {
            return false;
        }
    }

    public static void init() {
        setLEDS("on");
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0); //Sets the pipeline to 0
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0); //Sets the Limelight as a Vision Proccesor
    }

    public static void setPipeline(double pipeline) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline); //Sets the pipeline to pipeline
    }

    public static void disabled() {
        setLEDS("off"); // Turns off the god damm limelight cause im going to go blind and gouge my eyes out because wtf does it need to be so bright like holy hell
    }

    public static void setLEDS(String mode) {
        switch (mode) {
            case "off":
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1); // light off
                break;
            
            case "blink":
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(2); //light blinking
                break;
            
            case "on":
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3); // light on
                break;
            
            default:
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0); // as per pipeline mode (usually on)

        }
    }

}
