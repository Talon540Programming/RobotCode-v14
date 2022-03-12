package frc.robot.Modules;

import frc.robot.Robot;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AimFire {
  //Shooter, will probably be replaced with autonomous shooter stuff, need encoder
  public static void shooter() {
        if(Robot.controller.getRightBumper()) {
          Robot.shooterFly.set(ControlMode.PercentOutput, 1);
        } else {
          Robot.shooterFly.set(ControlMode.PercentOutput, 0);
        }
      }

  public static void centerAim(String target) {
  //https://i.kym-cdn.com/entries/icons/original/000/039/393/cover2.jpg
    switch(target) {
      case "top_hub": // TODO: check PID on Triclops
        Limelight.setPipeline(0);
        Limelight.getLimelightData();
        if(Math.abs(Limelight.nonZeroLimelightHorAng)>RobotInformation.deadbandAngle) { //Is the number within the deadband range?
          if(Limelight.nonZeroLimelightHorAng>0) { //Positive
            // Turn right
            double motorSpeed = (Math.abs(Limelight.nonZeroLimelightHorAng * .9)/59.6)+.05;
            motorSpeed = Math.round(motorSpeed * 100) / 100.0;
            MotorControl.oldDriveTrain(motorSpeed, motorSpeed);
          }
          if(Limelight.nonZeroLimelightHorAng<0) { //Negetive
            // Turn left
            double motorSpeed = (Math.abs(Limelight.nonZeroLimelightHorAng * .9)/59.6)+.05;
            motorSpeed = Math.round(motorSpeed * 100) / 100.0;
            MotorControl.oldDriveTrain(-motorSpeed, -motorSpeed);
          }
        }
        break;

      case "ball":

          break;
    }
  }

  public static void fire() { //TODO: One click fire

  }

}
