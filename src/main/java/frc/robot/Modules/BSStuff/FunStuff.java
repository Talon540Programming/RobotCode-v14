package frc.robot.Modules.BSStuff;

import frc.robot.Modules.MotorControl;
import frc.robot.Modules.Mechanisms.VisionSystems;
import frc.robot.Modules.Mechanisms.VisionSystems.Limelight.Limelight_Light_States;
import frc.robot.Modules.MotorControl.DriveCode;

public class FunStuff {
    public static enum Music {
        MrKrabs,

    }

    public static void RAVE_MODE() {
        VisionSystems.Limelight.setLEDS(Limelight_Light_States.blink);
        MotorControl.DriveCode.oldDriveTrain(0.6, -0.6);
    }

    public static void playMusic(Music sound) {
        switch(sound) {
            case MrKrabs:
                // Play MrKrabs Music
                break;
            
        }
    }
}