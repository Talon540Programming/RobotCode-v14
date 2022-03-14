package frc.robot.Modules;

import frc.robot.Robot;
import frc.robot.Modules.RobotInformation.FieldData.ValidTargets;
import frc.robot.Modules.RobotInformation;
import frc.robot.Modules.Mechanisms.VisionSystems;

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
  public static void centerAim(ValidTargets target) {
  //https://i.kym-cdn.com/entries/icons/original/000/039/393/cover2.jpg

    switch(target) {
      case upper_hub: // Center on the Retroreflector regardless of wether upper or lower cause they are stacked
      case lower_hub:
        VisionSystems.Limelight.setPipeline(0);
          if(Math.abs(VisionSystems.Limelight.nonZeroLimelightAngle)>RobotInformation.deadbandAngle) { //Is the number within the deadband range?
            if(VisionSystems.Limelight.nonZeroLimelightAngle>0) { //Positive
              // Turn right
              double motorSpeed = (Math.abs(VisionSystems.Limelight.nonZeroLimelightAngle * .9)/59.6)+.05;
              motorSpeed = Math.round(motorSpeed * 100) / 100.0;
              MotorControl.DriveCode.oldDriveTrain(motorSpeed, -motorSpeed);
            }
            if(VisionSystems.Limelight.nonZeroLimelightAngle<0) { //Negetive
              // Turn left
              double motorSpeed = (Math.abs(VisionSystems.Limelight.nonZeroLimelightAngle * .9)/59.6)+.05;
              motorSpeed = Math.round(motorSpeed * 100) / 100.0;
              MotorControl.DriveCode.oldDriveTrain(-motorSpeed, motorSpeed);
            }
          }
          // else {
          //   DriveCode.oldDriveTrain(0, 0);
          // }
        break;

      case ball:


          break;
    }
  }

/**
 * One click fire function
 */
  public static void fire(ValidTargets target) { //TODO: One click fire
    // if(Math.abs(Limelight.nonZeroLimelightHorAng)>RobotInformation.deadbandAngle) {
    //   centerAim(target);
    // } else if(Math.abs(Limelight.nonZeroLimelightHorAng)<RobotInformation.deadbandAngle) {
    // }


  }

}