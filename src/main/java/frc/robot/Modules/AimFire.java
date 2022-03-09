package frc.robot.Modules;

import frc.robot.Robot;
import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.Modules.RobotInformation;

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
        case "top_hub":
            Limelight.setPipeline(0);
            Limelight.getLimelightData();
            if(Math.abs(Limelight.nonZeroLimelightHorAng)>RobotInformation.deadbandAngle) { //Is the number within the deadband range?
                if(Limelight.nonZeroLimelightHorAng>0) { //Positive
                    // Turn right
                    double motorSpeed = (Math.abs(Limelight.nonZeroLimelightHorAng * .9)/59.6)+.05;
                    motorSpeed = Math.round(motorSpeed * 100) / 100.0;
                    Robot.drive.tankDrive(motorSpeed, -motorSpeed);
                }
                if(Limelight.nonZeroLimelightHorAng<0) { //Negetive
                    // Turn left
                    double motorSpeed = (Math.abs(Limelight.nonZeroLimelightHorAng * .9)/59.6)+.05;
                    motorSpeed = Math.round(motorSpeed * 100) / 100.0;
                    Robot.drive.tankDrive(-motorSpeed, motorSpeed);
                }
            }
            break;
        }
  }

    /** One click fire code
    public void fire() {
        double transferPercent = 0.35; 
        double[][] limelightData = getLimelightData();
        double idealAngle = limelightData[0][1];
        double idealVelocity = limelightData[0][2]; // flywheel diameter in meters

        double rpm = Flywheel.getRPM(idealVelocity, transferPercent);
        if (!intakeLimit.get()) {
            wrist.set(ControlMode.PercentOutput, 0.75);
        }
        Robot.shooterFly.set(ControlMode.Velocity, 4*rpm*2048); //TODO: Gear ratio multiplier MIGHT HAVE TO MULTIPLY BY 2048- will test later
        if ((gyro.getRoll() > idealAngle + 5) && !upperShooterLimit.get()) { // TODO: Adjust degrees of freedom +- 5
            hood.set(ControlMode.PercentOutput, 0.1);
            ready = false;
        }
        else if ((gyro.getRoll() < idealAngle-5) && !Robot.lowerShooterLimit.get()) {
            hood.set(ControlMode.PercentOutput, -0.1);
            ready = false;
        }
        else {
            ready = true;
        }
        if (ready) {
            rollers.set(ControlMode.PercentOutput, 1);
        }
        else {
            Robot.climbRotation.set(ControlMode.PercentOutput, 0);
        }
        //adjust to fine tuning 
        // double OneRPM = (60*idealVelocity)/(8*Math.PI); //RPM required if 100% of speed from flywheel is transferred to the ball
        // double actualRPM = 10*OneRPM; //RPM needed bearing 10% of velocity is transferred from flywheel to ball
        // double flywheelMotor = actualRPM/the maximum rpm of the motor; //The motor output in percentage of the flywheel motor
    } 
    */

    /** Hood Code
    public void hood() {
        if (Robot.controller.getXButton()) {
        hood.set(ControlMode.PercentOutput, 0.1);
        }
        else if (Robot.controller.getYButton()) {
        hood.set(ControlMode.PercentOutput, -0.1);
        }
        else { 
        hood.set(ControlMode.PercentOutput, 0);
        }
    }
    */

}
