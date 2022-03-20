/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Modules.MotorControl;
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
import edu.wpi.first.cscore.UsbCamera;
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
  private static final String shootFirst = "Shoot First";
  private static final String taxiFirst = "Taxi First";

  double rollerCounter;
  double flywheelCounter;

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
    gyro = new AHRS(SerialPort.Port.kUSB); //TODO: Change port based on where electrical says it is, if it takes too long, screw it and just use the commented out moveBack function in MotorControl.java

    // Robot Visual Control
    RobotLEDStateCounter = 0;
    GameControl.UserControl.currentRobotLEDState = RobotLEDState.off;
    
    CameraServer.startAutomaticCapture("Ball Camera", 0);
    
    // AUTO OPTIONS
    VisionSystems.Limelight.init();
    GameControl.initializeAllianceChooser();
  }
  
  @Override
  public void robotPeriodic() {
    // Display information relayed by Limelight and RPM information for testing
    VisionSystems.Limelight.updateSmartDashboard();

    double value = (104-RobotInformation.RobotData.RobotMeasurement.LimelightHeightInches)/(Math.tan(Math.toRadians(VisionSystems.Limelight.tx)+40));
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
    // TODO ISSUE: Rumbling during AUTO
    // Safety.batterySafety();
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

    SmartDashboard.putNumber("Wrist Value", wrist.getSensorCollection().getIntegratedSensorAbsolutePosition()); //TODO: Removed based on whether we need to sort out mechanical stop
  }

  @Override
  public void autonomousInit() {
    VisionSystems.Limelight.setLEDS(Limelight_Light_States.on);
    GameControl.currentMatchType = MatchTypes.auto;
    gyro.reset();
    rollerCounter = 0;
    flywheelCounter = 0;

  }

  @Override
  public void autonomousPeriodic() { 
    //TODO: If it can't make the shoot, as our angle is too high, we can shoot and THEN back up 5head (change the selected auto on the smartdashboard)
    //TODO: Check to make sure that we can see the upper hub with the limelight when we are on the tarmac. Else you have to remove the hubPresent clause :)

    // VisionSystems.BallTracking.updateCoprocessorValues(); // Update the Alliance Color Periodically for the Pi
    // VisionSystems.BallTracking.coprocessorErrorCheck(); // Check if the reporting Alliance and the Sent alliance are the same, if not run an error
    
    SmartDashboard.putNumber("Flywheel RPM: ", MotorControl.getRPM(Motors.Shooter));
    SmartDashboard.putNumber("Flywheel Velocity", MotorControl.getCurrentVelocity(shooterFly));
    if(Math.abs(VisionSystems.Limelight.tx)>RobotInformation.deadbandAngle) {
      AimFire.centerAim(ValidTargets.lower_hub);
    } else { // Check if we shot yet
      shooterFly.set(ControlMode.PercentOutput, 1); // Set flywheel to 100%
      flywheelCounter++; // Increase the time flywheel has been charged 1.5 seconds rn
      if(flywheelCounter>((2*1000)/(20/**current tick rate */))) { // if held for that long run the rollers
        rollers.set(ControlMode.PercentOutput, -1); // Run the rollers //TODO:FIND IF IT IS POSITIVE OR NEGEITVE
        rollerCounter++;
        if(rollerCounter > ((3*1000)/(20/**current tick rate */))) {
          shooterFly.set(ControlMode.PercentOutput, 0);
          if(VisionSystems.Limelight.getDistanceFromHubStack() < (RobotInformation.RobotData.RobotMeasurement.botlengthBumpersMeters+RobotInformation.FieldData.tarmacLengthMeters+1)) {
            MotorControl.DriveCode.oldDriveTrain(-0.3, -0.3);
          }
        }
      }
    }
    /** Dream Auto Code
    switch (m_autoSelected) {
      case (shootFirst) :
        //TODO: counter being 50 = 1 second. Adjust the timings as necessary :)
        if (counter < 54.0 * 4) {
          shooterFly.set(ControlMode.PercentOutput, 1); //TODO: adjust power based on arc needed
          AimFire.centerAim(ValidTargets.lower_hub);
        }
        if (counter > (42.2*3) && counter < (54.0 * 4)) {
          rollers.set(ControlMode.PercentOutput, 1);//TODO: Adjust power based on speed of shooting
        }
        else { // stop the motors after auto routine
          shooterFly.set(ControlMode.PercentOutput, 0);
          rollers.set(ControlMode.PercentOutput, 0);
          MotorControl.DriveCode.driveStraight(-0.1); //TODO: CHange power or switch to moveBackwards function if needed
        }
        break;

      case (taxiFirst) :
        //Taxi Code (Front Bumper needs to fully cross the tarmac)
        //TODO: Can switch from driveStraight to move Backwards if needed :)
        if(VisionSystems.Limelight.hubPresent() && (VisionSystems.Limelight.getDistanceFromHubStack()<(RobotInformation.RobotData.RobotMeasurement.botlengthMeters+RobotInformation.FieldData.tarmacLengthMeters+0.1))) { //If the top hub is present and we are less than 2.3 meters away drive backwards
          MotorControl.DriveCode.driveStraight(-0.1); //TODO: Adjust the power based on how fast it moves or whether it works
        } 
        else if(RobotInformation.RobotData.RobotMeasurement.botlengthMeters+RobotInformation.FieldData.tarmacLengthMeters <= VisionSystems.Limelight.getDistanceFromHubStack()) {
          drive.tankDrive(0,0); //stop moving
          //TODO: counter being 50 = 1 second. Adjust the timings as necessary :)
          if (counter < 54.0 * 4) {
            shooterFly.set(ControlMode.PercentOutput, 1); //TODO: adjust power based on arc needed
          }
          if (counter > (42.2*3) && counter < (54.0 * 4)) {
            rollers.set(ControlMode.PercentOutput, 1);//TODO: Adjust power based on speed of shooting
          }
          else { // stop the motors after auto routine
            shooterFly.set(ControlMode.PercentOutput, 0);
            rollers.set(ControlMode.PercentOutput, 0);
          }
        } 
        break;
    }
     */
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
        MotorControl.DriveCode.tankDrive(); //TODO: fix the inversion problems with tank drive. I don't want to do it on fricking GitHub editor so we can do it with testing :)
        MotorControl.FlywheelCode.flywheel();

      // 50 ticks * 20 ms = 1 second
      if(ControllerStatesCounter>((RobotInformation.DriveTeamInfo.teleopModeSwitchTimeout*100)/20)) { // Timeout in ms / tickrate
        if(controller.getStartButton() && controller.getBackButton()) { // TODO: Teach Ojas and Chirayu how controller modes work
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
        if(controller.getStartButton() && controller.getBackButton()) { // TODO: Teach Ojas and Chirayu how controller modes work
          GameControl.currentControllerState = ControllerStates.drive_mode;
          ControllerStatesCounter=0;
        }
      } else {
        ControllerStatesCounter++;
      }
      
    }
  
    // // Center bot on top hub
    // if(leftJoy.getRawButton(1)) {
    //   VisionSystems.Limelight.setLEDS(Limelight_Light_States.on);
    //   AimFire.centerAim(ValidTargets.upper_hub);
    // }

    // AimFire.shooter();
    // Intake.wrist();
    // Intake.rollers();
    // MotorControl.DriveCode.tankDrive(); //TODO: fix the inversion problems with tank drive. I don't want to do it on fricking GitHub editor so we can do it with testing :)
    // MotorControl.flywheel();
    // Climbers.climb();
    // Climbers.climbrotation();
  }

  @Override
  public void teleopExit() {

  }

}