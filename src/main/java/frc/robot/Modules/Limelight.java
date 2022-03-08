package frc.robot.Modules;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    public static double nonZeroLimelightHorAng; // Used to orient bot

    public static double[][] getLimelightData() {
        double[][] LimelightInfo = {{0,0,0},{0,0,0}};
        boolean targetPresence = hubPresent();
        if (targetPresence) {
          return LimelightInfo;
        } else {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        // double heightDifference = ((72/39.37)-(17/39.37)); //13.0 (photo room)
        double heightDifference = ((104/39.37)-(22.76/39.37)); //14.0 https://www.desmos.com/calculator/zmzfln2j6v
        // double fixedLLANGLE = 14.7734450937; //13.0 Angle
        double fixedLLANGLE = 40; //14.0 Angle https://www.desmos.com/calculator/zmzfln2j6v
    
        double horizontalAngle = table.getEntry("tx").getDouble(0);
        double verticalAngle = table.getEntry("ty").getDouble(0);
        double limelightLatency = table.getEntry("tl").getDouble(0);
    
        double distance = heightDifference / (Math.tan(((Math.toRadians(fixedLLANGLE)) + (Math.toRadians(verticalAngle)))));
        double angle = Math.toDegrees(Math.atan((Math.tan(Math.toRadians(-45)) * (distance)-(2 * heightDifference)) / (-distance)));
        double velocity = Math.sqrt(-1 * ((9.8 * distance * distance * (1 + (Math.pow(Math.tan(Math.toRadians(angle)), 2))) )/((2 * heightDifference)-(2 * distance * Math.tan(Math.toRadians(angle))))));
    
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
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0); //Sets the pipeline to 0
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0); //Sets the Limelight as a Vision Proccesor
    }

    public static void setPipeline(double pipeline) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline); //Sets the pipeline to pipeline
    }
}
