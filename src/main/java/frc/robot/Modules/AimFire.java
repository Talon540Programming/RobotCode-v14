package frc.robot.Modules;

import frc.robot.Robot;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AimFire {
/**
 * If the right bumper is pressed, run the shooter flywheel at full speed. Otherwise, stop it
 */
  public static void shooter() {
        if(Robot.controller.getRightBumper()) {
          Robot.shooterFly.set(ControlMode.PercentOutput, 1);
        } else {
          Robot.shooterFly.set(ControlMode.PercentOutput, 0);
        }
      }

/**
 * The function that will center the robot's aim on the target.
 * 
 * @param target The name of the target to aim at. ("top_hub","ball")
 */
  public static void centerAim(String target) {
  //https://i.kym-cdn.com/entries/icons/original/000/039/393/cover2.jpg
    switch(target.toLowerCase()) {
      case "top_hub": // TODO: check PID on Triclops
        Limelight.setPipeline(0);
        Limelight.getLimelightData();
        if(Math.abs(Limelight.nonZeroLimelightHorAng)>RobotInformation.deadbandAngle) { //Is the number within the deadband range?
          if(Limelight.nonZeroLimelightHorAng>0) { //Positive
            // Turn right
            double motorSpeed = (Math.abs(Limelight.nonZeroLimelightHorAng * .9)/59.6)+.05;
            motorSpeed = Math.round(motorSpeed * 100) / 100.0;
            DriveCode.oldDriveTrain(motorSpeed, motorSpeed);
          }
          if(Limelight.nonZeroLimelightHorAng<0) { //Negetive
            // Turn left
            double motorSpeed = (Math.abs(Limelight.nonZeroLimelightHorAng * .9)/59.6)+.05;
            motorSpeed = Math.round(motorSpeed * 100) / 100.0;
            DriveCode.oldDriveTrain(-motorSpeed, -motorSpeed);
          }
        } 
        // else { 
        //   DriveCode.oldDriveTrain(0, 0);
        // }
        break;

      case "ball":

          break;
    }
  }


/**
 * One click fire function
 */
  public static void fire() { //TODO: One click fire

  }

}
