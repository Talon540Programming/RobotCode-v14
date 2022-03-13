package frc.robot.Modules;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;


public class BallTracking {
/**
 * This function is called to initalise Alliance color and ball tracking
 */
    public static void maininit() {
        SmartDashboard.putString("Alliance Color", "Waiting for Alliance Color");
    }

/**
 * This function is called when auto first started. 
 * 
 * It sets the alliance color to the value of the alliance color selected in the SmartDashboard
 */
    public static void autoinit() {
        NetworkTableInstance.getDefault().getTable("TalonPi").getEntry("Alliance Color").setString(SmartDashboard.getString("Alliance Color", "RED"));
    }
}
