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
import frc.robot.Modules.PIDControl;
import frc.robot.Modules.AimFire;
import frc.robot.Modules.GameControl;
import frc.robot.Modules.RobotInformation;
import frc.robot.Modules.Safety;
import frc.robot.Modules.Mechanisms.Climbers;
import frc.robot.Modules.Mechanisms.Intake;
import frc.robot.Modules.Mechanisms.VisionSystems;
import frc.robot.Modules.GameControl.ControllerStates;
import frc.robot.Modules.GameControl.MatchTypes;
import frc.robot.Modules.GameControl.UserControl.RobotLEDState;
import frc.robot.Modules.Mechanisms.VisionSystems.Limelight.Limelight_Light_States;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Modules.RobotInformation.FieldData.ValidTargets;
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

  @Override
  public void robotInit() {
    // MOTORS
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

    // Follow master motors
    rightSlave.follow(rightMaster);
    leftSlave.follow(leftMaster);

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
    
    CameraServer.startAutomaticCapture("Ball Camera", 0);
    GameControl.UserControl.ledCounter = 0;
    GameControl.UserControl.oldLedValue = 0;
    
    // AUTO OPTIONS
    VisionSystems.Limelight.init();
    GameControl.initializeAllianceChooser();
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
    if(rightJoy.getRawButton(1)) {
      Robot.shooterFly.set(ControlMode.PercentOutput, 1);
    } else {
      Robot.shooterFly.set(ControlMode.PercentOutput, 0);
    }

    double current_velocity = MotorControl.getCurrentVelocity(shooterFly);
    double current_RPM = current_velocity/4/2048*60*10;

    SmartDashboard.putNumber("Max Velocity",RobotInformation.RobotData.MotorData.Shooter.Flywheel.maxVelocity);
    SmartDashboard.putNumber("Max RPM",RobotInformation.RobotData.MotorData.Shooter.Flywheel.maxRPM/4);
    SmartDashboard.putNumber("Flywheel Velocity", current_velocity);
    SmartDashboard.putNumber("Testing Flywheel RPM", current_RPM);

    SmartDashboard.putNumber("Wrist Value", wrist.getSensorCollection().getIntegratedSensorAbsolutePosition());

  }

  @Override
  public void autonomousInit() {
    VisionSystems.Limelight.setLEDS(Limelight_Light_States.on);
    GameControl.currentMatchType = MatchTypes.auto;
    gyro.reset();
    rollerCounter = 0;
    flywheelCounter = 0;
    ballFired = false;
    aimCentered = false;
  }

  @Override
  public void autonomousPeriodic() { 
    // VisionSystems.BallTracking.updateCoprocessorValues(); // Update the Alliance Color Periodically for the Pi
    // VisionSystems.BallTracking.coprocessorErrorCheck(); // Check if the reporting Alliance and the Sent alliance are the same, if not run an error
    
    SmartDashboard.putNumber("Flywheel RPM: ", MotorControl.getRPM(Motors.Shooter));

    if(!ballFired) {
      shooterFly.set(ControlMode.PercentOutput, 1);
      AimFire.centerAim(ValidTargets.upper_hub);
      flywheelCounter++;

      if(flywheelCounter > ((RobotInformation.RobotData.AutonomousData.flywheelPeriod*1000)/(20))) {
        rollers.set(ControlMode.PercentOutput, -1);
        rollerCounter++;

        if(rollerCounter > ((RobotInformation.RobotData.AutonomousData.rollersPeriod*1000)/(20))) {
          shooterFly.set(ControlMode.PercentOutput, 0);
          rollers.set(ControlMode.PercentOutput, 0);
          ballFired = true;
        }
      }

    } else if(ballFired) {
      MotorControl.DriveCode.driveToDistance(RobotInformation.RobotData.AutonomousData.taxiDriveDistance);
    }

  }

  @Override
  public void autonomousExit() {
    rollers.set(ControlMode.PercentOutput, 0);
    shooterFly.set(ControlMode.PercentOutput,0);
  }

  @Override
  public void teleopInit() {
    VisionSystems.Limelight.setLEDS(Limelight_Light_States.on);
    GameControl.currentMatchType = MatchTypes.teleop_drive;
    GameControl.currentControllerState = ControllerStates.drive_mode;
    ControllerStatesCounter=0;

  }

  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Flywheel RPM: ", MotorControl.getRPM(Motors.Shooter)); // Velocity is measured by Falcon Encoder in units/100ms. Convert to RPM by dividing by gear ratio (4) and encoder resolution of 2048. Then multiply by 600 to convert to per minute.

    // Drive Mode
    if(GameControl.currentControllerState == ControllerStates.drive_mode) {
        SmartDashboard.putString("Teleop Mode", "Drive Mode");

        // Center bot on top hub
        if(leftJoy.getRawButton(1)) {
          VisionSystems.Limelight.setLEDS(Limelight_Light_States.on);
          AimFire.centerAim(ValidTargets.upper_hub);
        }

        AimFire.shooter();
        Intake.runWrist();
        Climbers.climb();
        Intake.rollers();
        MotorControl.DriveCode.tankDrive();
        MotorControl.FlywheelCode.flywheel();

      // 50 ticks * 20 ms = 1 second
      if(ControllerStatesCounter>((RobotInformation.DriveTeamInfo.teleopModeSwitchTimeout*100)/20)) { // Timeout in ms / tickrate
        if(controller.getStartButton() && controller.getBackButton()) {
          GameControl.currentControllerState = ControllerStates.climb_mode;
          ControllerStatesCounter=0;
        }
      } else {
        ControllerStatesCounter++;
      }
    }
    
    // Climb Mode
    if(GameControl.currentControllerState == ControllerStates.climb_mode) {
        SmartDashboard.putString("Teleop Mode", "Climb Mode");
        
        // Climb Code
        Climbers.climb();
        Climbers.climbrotation();

      // 50 ticks * 20 ms = 1 second
      if(ControllerStatesCounter>((RobotInformation.DriveTeamInfo.teleopModeSwitchTimeout*100)/20)) { // Timeout in ms / tickrate
        if(controller.getStartButton() && controller.getBackButton()) {
          GameControl.currentControllerState = ControllerStates.drive_mode;
          ControllerStatesCounter=0;
        }
      } else {
        ControllerStatesCounter++;
      }
      
    }

    // PID TESTING
    PIDControl.driveDistanceTesting((4.5)); //TODO: TEST THIS
    PIDControl.limelightCenterTest(); //TODO: Test This
  }

  @Override
  public void teleopExit() {

  }

}