package frc.robot.Modules;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.Robot;
import frc.robot.Modules.GameControl.UserControl.rumbleSides;

public class Safety {
    public static PowerDistribution PDP = new PowerDistribution(0, ModuleType.kCTRE);

    public static void RaspberryPiSafety() {

    }

    public static void PowerDistributionPanelSafety() {
        double temp = PDP.getTemperature();
        double inputVoltage = PDP.getVoltage();
    }

    public static void batterySafety() {
        // Checking to see if the battery voltage is below a certain level. If it is, it will set the rumble to half on both sides.
        if(RobotController.getBatteryVoltage() < RobotInformation.DriveTeamInfo.safeBatteryLevel) {
            GameControl.UserControl.setControllerRumble(rumbleSides.both, 0.5);
            DriverStation.reportWarning("Batter Low Voltage Detected",false);
        }
    }
}
