/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Modules.MotorControl;
import frc.robot.Modules.GameControl;
import frc.robot.Modules.RobotInformation;
import frc.robot.Modules.Safety;
import frc.robot.Modules.Outreach.MGBOT_20;
import frc.robot.Modules.Mechanisms.VisionSystems;
import frc.robot.Modules.GameControl.ControllerStates;
import frc.robot.Modules.GameControl.MatchTypes;
import frc.robot.Modules.GameControl.UserControl.RobotLEDState;
import frc.robot.Modules.Mechanisms.VisionSystems.Limelight.Limelight_Light_States;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Modules.RobotInformation.RobotData.MotorData.motorTypes.Motors;

import com.kauailabs.navx.frc.AHRS;


public class Robot extends TimedRobot {
  // MOTOR VARIABLES
  public static WPI_TalonFX leftSlave, rightSlave, leftMaster, rightMaster; // Drivetrain Motors
  public static WPI_TalonFX climbExtension; // Climb Extension motor
  public static WPI_TalonFX shooterFly; // Flywheel motor
  public static WPI_TalonFX wrist; // Wrist motor
  public static WPI_TalonFX climbRotation; // Climb Rotation motor
  public static TalonSRX rollers; // Roller motor

  //CONTROLLERS
  public static Joystick leftJoy, rightJoy; //Used for tank drive and for other things
  public static XboxController controller; // Used for button man mechanism controls

  // SENSORS
  public static AHRS gyro; //9-axis-> used mainly to orient shooter hood using roll

  // MISCELLANEOUS
  public static DifferentialDrive drive; //Used for monitoring tank drive motion

  double rollerCounter;
  double flywheelCounter;
  public static boolean ballFired;
  public static boolean aimCentered;


  private static int ControllerStatesCounter;
  private static int RobotLEDStateCounter;

  private static MGBOT_20 outreachBot;

  @Override
  public void robotInit() {
    // DECLARE MOTORS
    //Declare Drivetrain motors
    rightMaster = new WPI_TalonFX(RobotInformation.RobotData.RobotPorts.DRIVETRAIN_FRONTRIGHT);
    rightSlave = new WPI_TalonFX(RobotInformation.RobotData.RobotPorts.DRIVETRAIN_BACKRIGHT);
    leftMaster = new WPI_TalonFX(RobotInformation.RobotData.RobotPorts.DRIVETRAIN_FRONTLEFT);
    leftSlave = new WPI_TalonFX(RobotInformation.RobotData.RobotPorts.DRIVETRAIN_BACKLEFT);

    // Declare Climb Motors
    climbRotation = new WPI_TalonFX(RobotInformation.RobotData.RobotPorts.CLIMBROTATION);
    climbExtension = new WPI_TalonFX(RobotInformation.RobotData.RobotPorts.CLIMBEXTENSION);

    // Declare Intake Motors
    wrist = new WPI_TalonFX(RobotInformation.RobotData.RobotPorts.INTAKE_WRIST);
    rollers = new TalonSRX(RobotInformation.RobotData.RobotPorts.INTAKE_ROLLERS);

    // Declare Shooter Motors
    shooterFly = new WPI_TalonFX(RobotInformation.RobotData.RobotPorts.SHOOTER_FLY);

    // Initalise motor values (Climbers, Flywheel)
    MotorControl.motor_init();

    // Used for tank and arcade drive respectively
    drive = new DifferentialDrive(leftMaster, rightMaster);

    // CONTROLLER PORTS
    leftJoy = new Joystick(0);
    rightJoy = new Joystick(1);
    controller = new XboxController(2);
    gyro = new AHRS(SerialPort.Port.kUSB);

    // Robot Visual Control
    RobotLEDStateCounter = 0;
    GameControl.UserControl.currentRobotLEDState = RobotLEDState.off;
    VisionSystems.intakeVision.init();

    GameControl.UserControl.ledCounter = 0;
    GameControl.UserControl.oldLedValue = 0;
    
    // AUTO OPTIONS
    VisionSystems.Limelight.init();
    GameControl.initializeAllianceChooser();

    outreachBot = new MGBOT_20(1, 1, 1, 1);

  }
  
  @Override
  public void robotPeriodic() {
    // Display information relayed by Limelight and RPM information for testing
    VisionSystems.Limelight.updateSmartDashboard();

    double value = (104-RobotInformation.RobotData.RobotMeasurement.LimelightHeightInches)/(Math.tan(Math.toRadians(VisionSystems.Limelight.tx)+Math.toRadians(RobotInformation.RobotData.RobotMeasurement.LimelightAngleDegrees)));
    SmartDashboard.putNumber("TESTING VALUE", value);

    if(RobotController.getUserButton()) { // Sequence is off -> on -> blink
      if(RobotLEDStateCounter > 1000/20) { // If the led is off and the button was not clicked in the last second
        switch(GameControl.UserControl.currentRobotLEDState) {
          case off:
            GameControl.UserControl.setRobotLEDS(RobotLEDState.on);
            break;
          case on:
            GameControl.UserControl.setRobotLEDS(RobotLEDState.blink);
            // Set blink
            break;
          case blink:
            GameControl.UserControl.setRobotLEDS(RobotLEDState.off);
            // Set off
            break;
        }
        RobotLEDStateCounter=0;
      }
    }
    
    RobotLEDStateCounter++; // Increase the counter
    SmartDashboard.putNumber("Current LED Counter", RobotLEDStateCounter); // REMOVEME
    GameControl.UserControl.setRobotLEDS(GameControl.UserControl.currentRobotLEDState); //Update the LED state to the current LED state
  
    /** SAFETY FIRST!!! ... */ 
    Safety.batterySafety();
  }

  @Override
  public void disabledPeriodic() {
    VisionSystems.Limelight.disabled(); // Turns off the limelight when robot is disabled among other things
    GameControl.currentMatchType = MatchTypes.disabled;
  }

  @Override
  public void testPeriodic() {

  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() { 
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    VisionSystems.Limelight.setLEDS(Limelight_Light_States.on);
    GameControl.currentMatchType = MatchTypes.teleop_drive;
    GameControl.currentControllerState = ControllerStates.drive_mode;
    ControllerStatesCounter = 0;
  }

  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Flywheel RPM: ", MotorControl.getRPM(Motors.Shooter)); // Velocity is measured by Falcon Encoder in units/100ms. Convert to RPM by dividing by gear ratio (4) and encoder resolution of 2048. Then multiply by 600 to convert to per minute.
    
    outreachBot.MGBOTClimbSystem();
    outreachBot.MGBOTDriveSystem();
    outreachBot.MGBOTShooterSystem();
    
  }

  @Override
  public void teleopExit() {

  }

}