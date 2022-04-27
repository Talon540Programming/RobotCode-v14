package frc.robot.Modules.BS_Stuff;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Robot;

public class MGBOT_20 {
    private static double driveMultiplier = 0;
    private static double shooterMultiplier = 0;
    private static double wristMultiplier = 0;
    private static double rollerMultiplier = 0;


    /**
     * @param dm Drive multiplier
     * @param sm Shooter multiplier
     * @param wm Wrist multiplier
     * @param rm Roller multiplier
     */
    public MGBOT_20(double dm, double sm, double wm, double rm) {
        driveMultiplier = dm;
        shooterMultiplier = sm;
        wristMultiplier = wm;
        rollerMultiplier = rm;
    }

/**
 * If the joystick is moved more than 20% in either direction, drive the robot
 */
    private static void tankDrive() {
        if ((Math.abs(Robot.rightJoy.getY()) > 0.2) || Math.abs(Robot.leftJoy.getY()) > 0.2) {
            Robot.drive.tankDrive((-Robot.leftJoy.getY() * driveMultiplier), ( Robot.rightJoy.getY() * driveMultiplier));
        }
    }
    
/**
 * This function is called in the main loop of the program and it calls the tankDrive function
 */
    public void MGBOTDriveSystem() {
        tankDrive();
    }
    
/**
 * If the right bumper is pressed, set the shooter motor to 1 * the shooterMultiplier. If the right
 * bumper is not pressed, set the shooter motor to 0 * the shooterMultiplier
 */
    private static void shooter() {
        if(Robot.controller.getRightBumper()) {
          Robot.shooterFly.set(ControlMode.PercentOutput, 1 * shooterMultiplier);
        } else {
          Robot.shooterFly.set(ControlMode.PercentOutput, 0 * shooterMultiplier);
        }
    }

/**
 * If the left joystick is moved more than 20% in the Y direction, move the wrist motor at a speed
 * equal to the joystick's Y position multiplied by the wristMultiplier variable
 */
    private static void runWrist() {
        if(Math.abs(Robot.controller.getLeftY()) > 0.2) {
            Robot.wrist.set(ControlMode.PercentOutput, (Robot.controller.getLeftY() * -wristMultiplier));
        } else {
            Robot.wrist.set(ControlMode.PercentOutput, 0);
        }
    }

/**
 * If the left bumper is pressed, the rollers spin inwards, if the A button is pressed, the rollers
 * spin outwards, if the B button is pressed, the rollers spin inwards, and if none of the above are
 * pressed, the rollers stop
 */
    private static void runRollers() { // Had absolute value of getRightY, would never be below zero!
        if (Robot.controller.getLeftBumper()) { //Right trigger spins intake from field to robot
            Robot.rollers.set(ControlMode.PercentOutput, -(rollerMultiplier));
        } else if(Robot.controller.getAButton()) { //Left trigger spins intake from entrance of robot to flywheel
            Robot.rollers.set(ControlMode.PercentOutput, -rollerMultiplier);
        } else if(Robot.controller.getBButton()) { //Left trigger spins intake from flywheel to entrance of robot
            Robot.rollers.set(ControlMode.PercentOutput, rollerMultiplier);
        } else {
            Robot.rollers.set(ControlMode.PercentOutput, 0);
        }
    }
  
/**
 * This function runs the shooter, wrist, and rollers
 */
    public void MGBOTShooterSystem() {
        shooter();
        runWrist();
        runRollers();
    }

/**
 * If the D pad is pressed up, the motor will run at 60% power. If the D pad is pressed down, the motor
 * will run at -60% power. If the D pad is not pressed, the motor will not run
 */
    private static void climb() {
        if (Robot.controller.getPOV() == 0) { //Up on D pad
            Robot.climbExtension.set(ControlMode.PercentOutput, 0.6);
        } else if (Robot.controller.getPOV() == 180) { //Down on D pad
            Robot.climbExtension.set(ControlMode.PercentOutput, -0.6);
        } else {
            Robot.climbExtension.set(ControlMode.PercentOutput, 0);
        }
    }

/**
 * If the D pad is pressed right, the motor will go forward, if the D pad is pressed left, the motor
 * will go backwards, and if the D pad is not pressed, the motor will stop
 */
    private static void climbrotation() {
        if (Robot.controller.getPOV() == 90) { // Right on the D pad
            Robot.climbRotation.set(ControlMode.PercentOutput, 1);
        } else if (Robot.controller.getPOV() == 270) { //Left on the D pad
            Robot.climbRotation.set(ControlMode.PercentOutput, -1);
        } else {
            Robot.climbRotation.set(ControlMode.PercentOutput, 0);
        }
    }

/**
 * It climbs the robot.
 */
    public void MGBOTClimbSystem() {
        climb();
        climbrotation();
    }
}
