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
import frc.robot.Modules.Mechanisms.Climbers;
import frc.robot.Modules.Mechanisms.Intake;
import frc.robot.Modules.Mechanisms.VisionSystems;
import frc.robot.Modules.GameControl.ControllerStates;
import frc.robot.Modules.GameControl.MatchTypes;
import frc.robot.Modules.GameControl.UserControl.rumbleSides;
import frc.robot.Modules.Mechanisms.VisionSystems.Limelight.Limelight_Light_States;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Modules.RobotInformation.FieldData.ValidTargets;
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
  private static final SendableChooser<String> m_chooser = new SendableChooser<>();
  private String m_autoSelected;
  private double counter;

  @Override
  public void robotPeriodic() {
    // Display information relayed by Limelight and RPM information for testing
    VisionSystems.Limelight.updateSmartDashboard();

    // Checking to see if the battery voltage is below a certain level. If it is, it will set the rumble to half on both sides.
    if(RobotController.getBatteryVoltage() < RobotInformation.DriveTeamInfo.safeBatteryLevel) {
      GameControl.UserControl.setControllerRumble(rumbleSides.both, 0.5);
      DriverStation.reportWarning("Batter Low Voltage Detected",false);
    }
  }

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

    // AUTO OPTIONS
    VisionSystems.Limelight.init();
    GameControl.initializeAllianceChooser();
    m_chooser.setDefaultOption("Default Auto", "SELECT AUTO!");
    m_chooser.addOption("Shoot First", shootFirst);
    m_chooser.addOption("Taxi First", taxiFirst);
    SmartDashboard.putData("Auto Choices", m_chooser);
  }

  //TODO: Remove this when not needed
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
    m_autoSelected = m_chooser.getSelected();
    gyro.reset();
    counter = 0;
  }

  @Override
  public void autonomousPeriodic() { 
    //TODO: If it can't make the shoot, as our angle is too high, we can shoot and THEN back up 5head (change the selected auto on the smartdashboard)
    //TODO: Check to make sure that we can see the upper hub with the limelight when we are on the tarmac. Else you have to remove the hubPresent clause :)

    switch (m_autoSelected) {
      case (shootFirst) :
        VisionSystems.BallTracking.updateCoprocessorValues(); // Update the Alliance Color Periodically for the Pi
        VisionSystems.BallTracking.coprocessorErrorCheck(); // Check if the reporting Alliance and the Sent alliance are the same, if not run an error
        SmartDashboard.putNumber("Flywheel RPM: ", shooterFly.getSelectedSensorVelocity()/4/2048*60*10);

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
        VisionSystems.BallTracking.updateCoprocessorValues(); // Update the Alliance Color Periodically for the Pi
        VisionSystems.BallTracking.coprocessorErrorCheck(); // Check if the reporting Alliance and the Sent alliance are the same, if not run an error
        SmartDashboard.putNumber("Flywheel RPM: ", shooterFly.getSelectedSensorVelocity()/4/2048*60*10);

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
  }

  @Override
  public void autonomousExit() { // Run apon exiting auto
    VisionSystems.Limelight.setLEDS(Limelight_Light_States.off);
  }

  @Override
  public void teleopInit() {
    VisionSystems.Limelight.setLEDS(Limelight_Light_States.on);
    GameControl.currentMatchType = MatchTypes.teleop_drive;
    GameControl.currentControllerState = ControllerStates.drive_mode;

  }

  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Flywheel RPM: ", shooterFly.getSelectedSensorVelocity()/4/2048*60*10); // Velocity is measured by Falcon Encoder in units/100ms. Convert to RPM by dividing by gear ratio (4) and encoder resolution of 2048. Then multiply by 600 to convert to per minute.

    if(GameControl.currentControllerState == ControllerStates.drive_mode) {
        // Driver aims to top hub or to balls
        if(leftJoy.getRawButton(1)) {
          VisionSystems.Limelight.setLEDS(Limelight_Light_States.on);
          AimFire.centerAim(ValidTargets.upper_hub);
        }

        AimFire.shooter();
        Intake.wrist();
        Intake.rollers();
        MotorControl.DriveCode.tankDrive(); //TODO: fix the inversion problems with tank drive. I don't want to do it on fricking GitHub editor so we can do it with testing :)
        MotorControl.flywheel();

      if(controller.getStartButton() && controller.getBackButton()) { // TODO: Teach Ojas and Chirayu how controller modes work
        GameControl.currentControllerState = ControllerStates.climb_mode;
      }
    }

    if(GameControl.currentControllerState == ControllerStates.climb_mode) {
        Climbers.climb();
        Climbers.climbrotation();

      if(controller.getStartButton() && controller.getBackButton()) {
        GameControl.currentControllerState = ControllerStates.drive_mode;
      }
    }
  }

  @Override
  public void teleopExit() { // Run apon exiting teleop
    VisionSystems.Limelight.setLEDS(Limelight_Light_States.off);
  }

  @Override
  public void disabledPeriodic() { // Run when in Disabled mode
    VisionSystems.Limelight.disabled(); // Turns off the limelight when robot is disabled among other things
    GameControl.currentMatchType = MatchTypes.disabled;
  }

}

/**
 * IterativeRobotBase implements a specific type of robot program framework, extending the RobotBase
 * class.
 *
 * <p>The IterativeRobotBase class does not implement startCompetition(), so it should not be used
 * by teams directly.
 *
 * <p>This class provides the following functions which are called by the main loop,
 * startCompetition(), at the appropriate times:
 *
 * <p>robotInit() -- provide for initialization at robot power-on
 *
 * <p>init() functions -- each of the following functions is called once when the appropriate mode
 * is entered:
 *
 * <ul>
 *   <li>disabledInit() -- called each and every time disabled is entered from another mode
 *   <li>autonomousInit() -- called each and every time autonomous is entered from another mode
 *   <li>teleopInit() -- called each and every time teleop is entered from another mode
 *   <li>testInit() -- called each and every time test is entered from another mode
 * </ul>
 *
 * <p>periodic() functions -- each of these functions is called on an interval:
 *
 * <ul>
 *   <li>robotPeriodic()
 *   <li>disabledPeriodic()
 *   <li>autonomousPeriodic()
 *   <li>teleopPeriodic()
 *   <li>testPeriodic()
 * </ul>
 *
 * <p>final() functions -- each of the following functions is called once when the appropriate mode
 * is exited:
 *
 * <ul>
 *   <li>disabledExit() -- called each and every time disabled is exited
 *   <li>autonomousExit() -- called each and every time autonomous is exited
 *   <li>teleopExit() -- called each and every time teleop is exited
 *   <li>testExit() -- called each and every time test is exited
 * </ul>
 */