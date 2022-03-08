package frc.robot.Modules;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;


public class BallTracking {
    public static void maininit() {
        SmartDashboard.putString("Alliance Color", "Waiting for Alliance Color");
    }

    public static void autoinit() {
        NetworkTableInstance.getDefault().getTable("TalonPi").getEntry("Alliance Color").setString(SmartDashboard.getString("Alliance Color", "RED"));
    }
}
