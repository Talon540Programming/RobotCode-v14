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
import frc.robot.Modules.Limelight;
import frc.robot.Modules.MotorControl;
import frc.robot.Modules.AimFire;
import frc.robot.Modules.DriveCode;
import frc.robot.Modules.BallTracking;
import frc.robot.Modules.Climbers;
import frc.robot.Modules.Intake;
import frc.robot.Modules.RobotInformation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
// import com.kauailabs.navx.frc.AHRS;


public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default"; //types of autos- not used
  private static final String kCustomAuto = "My Auto"; //types of autos- not currently used
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // MOTOR VARIABLES
  public static WPI_TalonFX leftSlave, rightSlave, leftMaster, rightMaster; //Falcon 500s
  public static WPI_TalonFX climbExtension;
  public static WPI_TalonFX shooterFly;
  public static WPI_TalonFX wrist;
  public static WPI_TalonFX climbRotation;
  public static TalonSRX rollers;

  //CONTROLLERS
  public static Joystick leftJoy, rightJoy; //Used for tank drive
  public static XboxController controller; // Used for button man mechanism controls

  // SENSORS
  // private AHRS gyro; //9-axis-> used mainly to orient shooter hood using roll

  // private Encoder rightEncoder, leftEncoder; // Drivetrain encoders (might just use integrated Falcon stuff who knows)
  // private DigitalInput lowerShooterLimit, upperShooterLimit; // Limit switch to reset hood to its default position
  // private DigitalInput intakeLimit; // Limit switch to know if intake is ready to kickup

  // MISCELLANEOUS
  public static DifferentialDrive drive; //Used for monitoring tank drive motion

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

    // Used to select autonomous mode
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    Limelight.init();
    BallTracking.maininit(); //TODO: create sendableChooser for alliance COLOR
  }

  @Override
  public void testInit() {

  }

  @Override
  public void testPeriodic() {

  }

  @Override
  public void autonomousInit() {
    // m_autoSelected = m_chooser.getSelected();
    // System.out.println("Auto selected: " + m_autoSelected);
    Limelight.setLEDS("on");
  }

  @Override
  public void autonomousPeriodic() { //Two ball auto in theory //TODO: Write autocode
    BallTracking.autoinit();
    // Display information relayed by Limelight and RPM information for testing
    double[][] shooterCalculations = Limelight.getLimelightData();
    Limelight.updateSmartDashboard();

    SmartDashboard.putNumber("Flywheel RPM: ", shooterFly.getSelectedSensorVelocity()/4 * 2048);
    //SmartDashboard.putNumber("Current Angle", gyro.getRoll());

    //Taxi Code (Front Bumper needs to fully cross the tarmac)
    if(Limelight.hubPresent() && (shooterCalculations[0][0]<(RobotInformation.RobotData.RobotMeasurement.botlengthMeters+RobotInformation.FeildData.tarmacLengthMeters+0.1))) { //If the top hub is present and we are less than 2.3 meters away drive backwards
      drive.tankDrive(-0.1, -0.1);
    } else if(Limelight.hubPresent() && (shooterCalculations[0][0]>(RobotInformation.RobotData.RobotMeasurement.botlengthMeters+RobotInformation.FeildData.tarmacLengthMeters+0.6))) { //If we overshoot the target
      drive.tankDrive(0.1, 0.1);
    } else if(RobotInformation.RobotData.RobotMeasurement.botlengthMeters+RobotInformation.FeildData.tarmacLengthMeters < shooterCalculations[0][0] && shooterCalculations[0][0] <  (RobotInformation.RobotData.RobotMeasurement.botlengthMeters+RobotInformation.FeildData.tarmacLengthMeters + 0.2)) {
      // shoot
    } else {
      drive.tankDrive(0,0);
    }

  }

  @Override
  public void autonomousExit() { // Run apon exiting auto
    Limelight.setLEDS("off");
  }

  @Override
  public void teleopInit() {
    Limelight.setLEDS("on");
  }

  @Override
  public void teleopPeriodic() {
    Limelight.updateSmartDashboard();

    SmartDashboard.putNumber("Flywheel RPM: ", shooterFly.getSelectedSensorVelocity()/4 * 2048);
    // SmartDashboard.putNumber("Current Angle", gyro.getRoll());

    // Driver aims to top hub or to balls
    if(leftJoy.getRawButton(1)) { //center robot on top hub (retro reflector) // Changed to button, not trigger (left front button) //TODO: test PID loop
      Limelight.setLEDS("on");
      AimFire.centerAim("top_hub");
    }

    Climbers.climb();
    Climbers.climbrotation();
    AimFire.shooter();
    Intake.wrist();
    Intake.rollers();
    DriveCode.tankDrive();
    MotorControl.flywheel();
  }

  @Override
  public void teleopExit() { // Run apon exiting teleop
    Limelight.setLEDS("off");
  }

  @Override
  public void disabledPeriodic() { // Run when in Disabled mode
    Limelight.disabled(); // Turns off the limelight when robot is disabled among other things
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