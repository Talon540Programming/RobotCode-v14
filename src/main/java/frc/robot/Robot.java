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
import frc.robot.Modules.GameControl.UserControl.rumbleSides;
import frc.robot.Modules.Mechanisms.VisionSystems.Limelight.Limelight_Light_States;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;

import java.sql.DriverManager;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.Modules.RobotInformation.RobotData.MotorData.motorTypes.MotorPositions;
import frc.robot.Modules.RobotInformation.FieldData.ValidTargets;;
// import com.kauailabs.navx.frc.AHRS;


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
  // private AHRS gyro; //9-axis-> used mainly to orient shooter hood using roll

  // private Encoder rightEncoder, leftEncoder; // Drivetrain encoders (might just use integrated Falcon stuff who knows)
  // private DigitalInput lowerShooterLimit, upperShooterLimit; // Limit switch to reset hood to its default position
  // private DigitalInput intakeLimit; // Limit switch to know if intake is ready to kickup

  // MISCELLANEOUS
  public static DifferentialDrive drive; //Used for monitoring tank drive motion

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

    VisionSystems.Limelight.init();
    GameControl.initializeAllianceChooser();
  }

  @Override
  public void testInit() {
    SmartDashboard.putNumber("Test RPM", 0);
  }

  @Override
  public void testPeriodic() {
    double testRPM = SmartDashboard.getNumber("Test RPM", 0);
    // MotorControl.setRPM(MotorPositions.Shooter, 10);
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


    // PIDController FlywheelPIDController = new PIDController(RobotInformation.PID_Values.flywheel.kP, RobotInformation.PID_Values.flywheel.kI, RobotInformation.PID_Values.flywheel.kD);
    // FlywheelPIDController.calculate(shooterFly.getSensorCollection().getIntegratedSensorVelocity());
    // FlywheelPIDController.close();
    SmartDashboard.putNumber("Wrist Value", wrist.getSensorCollection().getIntegratedSensorAbsolutePosition()); //TODO: We're using a mehanical stop so we can delete this.
  }

  @Override
  public void testExit() {

  }

  @Override
  public void autonomousInit() {
    // m_autoSelected = m_chooser.getSelected();
    // System.out.println("Auto selected: " + m_autoSelected);
    VisionSystems.Limelight.setLEDS(Limelight_Light_States.on);
  }

  @Override
  public void autonomousPeriodic() { //Two ball auto in theory //TODO: Write autocode
    //TODO: Test the RPM we need at that specific distance
    //TODO: If it can't make the shoot, as our angle is too high, we can shoot and THEN back up 5head.
    VisionSystems.BallTracking.updateAllianceColor(); // Update the Alliance Color Periodically for the Pi
    VisionSystems.BallTracking.coprocessorErrorCheck(); // Check if the reporting Alliance and the Sent alliance are the same, if not run an error

    SmartDashboard.putNumber("Flywheel RPM: ", shooterFly.getSelectedSensorVelocity()/4/2048*60*10);
    //SmartDashboard.putNumber("Current Angle", gyro.getRoll());

    //Taxi Code (Front Bumper needs to fully cross the tarmac)
    if(VisionSystems.Limelight.hubPresent() && (VisionSystems.Limelight.getDistanceFromHubStack()<(RobotInformation.RobotData.RobotMeasurement.botlengthMeters+RobotInformation.FieldData.tarmacLengthMeters+0.1))) { //If the top hub is present and we are less than 2.3 meters away drive backwards
      drive.tankDrive(-0.1, -0.1); //TODO: need to create moveBackwards function that uses gyro and encoders to move back the appropriate distance. 
    } else if(VisionSystems.Limelight.hubPresent() && (VisionSystems.Limelight.getDistanceFromHubStack()>(RobotInformation.RobotData.RobotMeasurement.botlengthMeters+RobotInformation.FieldData.tarmacLengthMeters+0.6))) { //If we overshoot the target
      drive.tankDrive(0.1, 0.1); //TODO: might not want to move forward otherwise we'll need PID- it can stop fast enough
    } else if(RobotInformation.RobotData.RobotMeasurement.botlengthMeters+RobotInformation.FieldData.tarmacLengthMeters < VisionSystems.Limelight.getDistanceFromHubStack() && VisionSystems.Limelight.getDistanceFromHubStack() <  (RobotInformation.RobotData.RobotMeasurement.botlengthMeters+RobotInformation.FieldData.tarmacLengthMeters + 0.2)) {
      // TODO: Use the shooting code made from testing. Accidentally deleted it but I'll (Aryan) will put it in.
    } else {
      drive.tankDrive(0,0);
    }

  }

  @Override
  public void autonomousExit() { // Run apon exiting auto
    VisionSystems.Limelight.setLEDS(Limelight_Light_States.off);
  }

  @Override
  public void teleopInit() {
    VisionSystems.Limelight.setLEDS(Limelight_Light_States.on);
  }

  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Flywheel RPM: ", shooterFly.getSelectedSensorVelocity()/4/2048*60*10); // Velocity is measured by Falcon Encoder in units/100ms. Convert to RPM by dividing by gear ratio (4) and encoder resolution of 2048. Then multiply by 600 to convert to per minute.
    // SmartDashboard.putNumber("Current Angle", gyro.getRoll());

    // Driver aims to top hub or to balls
    if(leftJoy.getRawButton(1)) {
      VisionSystems.Limelight.setLEDS(Limelight_Light_States.on);
      AimFire.centerAim(ValidTargets.upper_hub);
    }

    Climbers.climb();
    Climbers.climbrotation();
    AimFire.shooter();
    Intake.wrist();
    Intake.rollers();
    MotorControl.DriveCode.tankDrive(); //TODO: fix the inversion problems with tank drive. I don't want to do it on fricking GitHub editor so we can do it with testing :)
    MotorControl.flywheel();
  }

  @Override
  public void teleopExit() { // Run apon exiting teleop
    VisionSystems.Limelight.setLEDS(Limelight_Light_States.off);
  }

  @Override
  public void disabledPeriodic() { // Run when in Disabled mode
    VisionSystems.Limelight.disabled(); // Turns off the limelight when robot is disabled among other things
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