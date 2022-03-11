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
import frc.robot.Modules.MotorContol;
import frc.robot.Modules.AimFire;
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
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
// import com.kauailabs.navx.frc.AHRS;


public class Robot extends TimedRobot {
  // AUTONOMOUS VARIABLES
  private int counter = 0;// TODO: Simple initialization for autonomous duration it takes to kickup counter-> can be adjusted on line 176
  private int counter2 = 0;
  private boolean ready, ready2; // Simple flag used for autonomous staging
  private static final String kDefaultAuto = "Default"; //types of autos- not used
  private static final String kCustomAuto = "My Auto"; //types of autos- not currently used
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private boolean stage1;  // auto staging

  // MOTOR VARIABLES
  private WPI_TalonFX leftSlave, rightSlave, leftMaster, rightMaster; //Falcon 500s
  public static WPI_TalonFX climbExtension;
  public static WPI_TalonFX shooterFly;
  public static WPI_TalonFX wrist;
  public static WPI_TalonFX climbRotation;
  public static TalonSRX rollers;

  //CONTROLLERS
  public Joystick leftJoy, rightJoy; //Used for tank drive
  public static XboxController controller; // Used for button man mechanism controls

  // SENSORS
  //private AHRS gyro; //9-axis-> used mainly to orient shooter hood using roll

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

    //Shooter Motors
    shooterFly = new WPI_TalonFX(RobotInformation.RobotData.RobotPorts.SHOOTER_FLY); 

    // Set all motor behavior to default settings to remove recidule calls also used for future config changes (brake mode)

    leftSlave.configFactoryDefault();
    rightSlave.configFactoryDefault();
    leftMaster.configFactoryDefault();
    rightMaster.configFactoryDefault();
    climbExtension.configFactoryDefault();
    shooterFly.configFactoryDefault();
    wrist.configFactoryDefault();
    climbRotation.configFactoryDefault();
    rollers.configFactoryDefault();

    // Follow master motors
    rightSlave.follow(rightMaster);
    leftSlave.follow(leftMaster);

    // Configure Shooter Flywheel
    shooterFly.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 1); // Tells to use in-built falcon 500 sensor
    shooterFly.configNominalOutputForward(0, 1);
    shooterFly.configNominalOutputReverse(0, 1);
    shooterFly.configPeakOutputForward(1, 1);
    shooterFly.configPeakOutputReverse(-1, 1);
    shooterFly.setInverted(false);
    shooterFly.setSensorPhase(false);
    // config P,I,D,F values- start by doubling F, then P, then D, then I (middle values) then increase/decrease over time
    // shooterFly.config_kF(0, 0.007, 1); // (F) Feed Forward Term
    // shooterFly.config_kP(0, 0.8192, 1); // (P) Proportional Term
    // shooterFly.config_kI(0, 0.0008, 1); // (I) Integral term
    // shooterFly.config_kD(0, 0.0256, 1); // (D) Differentiable Term
    
    // Set Climb rotation to be in brake mode by default
    climbRotation.setNeutralMode(NeutralMode.Brake);
    climbExtension.setNeutralMode(NeutralMode.Brake);

    // Used for tank and arcade drive respectively
    drive = new DifferentialDrive(leftMaster, rightMaster);

    // AUTO AND TELEOP STAGING FLAGS
    stage1 = false; // Auto starts in stage 1 and turns into stage 2
    ready = false; // Not ready to shoot by default
    ready2 = false;
    counter = 0;
    counter2 = 0;

    // CONTROLLER PORTS
    leftJoy = new Joystick(0);
    rightJoy = new Joystick(1);
    controller = new XboxController(2);

    // Used to select autonomous mode
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    // SENSOR INITIALIZATIONS
    //gyro = new AHRS(SerialPort.Port.kMXP); //SUBJECT TO CHANGE FROM ELECTRICAL COULD BE SOURCE OF ERROR
    // ALL Ports subject to change from Electrical
    //leftEncoder = new Encoder(0, 1);
    // leftEncoder.setDistancePerPulse(drive_dpp);
    // rightEncoder = new Encoder(2, 3);
    // rightEncoder.setDistancePerPulse(drive_dpp);
    // // Limit switches- CHANGE PORTS BASED ON ELECTRICAL
    // lowerShooterLimit = new DigitalInput(4);
    // upperShooterLimit = new DigitalInput(5);
    // intakeLimit = new DigitalInput(6);

    // Set integrated sensor position to 0 for encoder use
    // leftMaster.getSensorCollection().setIntegratedSensorPosition(0, 10);
    // rightMaster.getSensorCollection().setIntegratedSensorPosition(0, 10);

    Limelight.init();
    BallTracking.maininit(); //TODO: create sendableChooser for alliance COLOR
  }

  @Override
  public void testInit() {
    SmartDashboard.putNumber("Target RPM", 0);
    SmartDashboard.putNumber("Target Velocity", 0);
    SmartDashboard.putNumber("Target Angle", 0);
  }

  @Override
  public void testPeriodic() {
    double[][] shooterCalculations = Limelight.getLimelightData();
    Limelight.updateSmartDashboard();
    SmartDashboard.putNumber("Flywheel RPM: ", shooterFly.getSelectedSensorVelocity()/4 * 2048);

    //SmartDashboard.putNumber("Current Angle", gyro.getRoll());

    double targetrpm = SmartDashboard.getNumber("Target RPM", 0);
    double targetvelocity = SmartDashboard.getNumber("Target Velocity", 0);
    double targetangle = SmartDashboard.getNumber("Target Angle", 0);

    if (controller.getRightBumper()) { // sets shooter to the target rpm to test PID loop
      shooterFly.set(ControlMode.Velocity, 4*targetrpm*2048); //TODO: Gear ratio multiplier MIGHT HAVE TO MULTIPLY BY 2048- will test later
    }

    else if (controller.getLeftBumper()) { // sets shooter to the target velocity for ball shooting
      double rpm = MotorContol.getRPM(targetvelocity, 1);
      shooterFly.set(ControlMode.Velocity, rpm);
    }

  }

  @Override
  public void autonomousInit() {
    // m_autoSelected = m_chooser.getSelected();
    // System.out.println("Auto selected: " + m_autoSelected);
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

  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    double[][] shooterCalculations = Limelight.getLimelightData();
    Limelight.updateSmartDashboard();

    SmartDashboard.putNumber("Flywheel RPM: ", shooterFly.getSelectedSensorVelocity()/4 * 2048);
    // SmartDashboard.putNumber("Current Angle", gyro.getRoll());

    // DRIVE CALL
    if ((Math.abs(rightJoy.getY()) > 0.2) || Math.abs(leftJoy.getY()) > 0.2) {
      drive.tankDrive((-rightJoy.getY() * RobotInformation.DriveTeamInfo.driverPercentage), (leftJoy.getY() * RobotInformation.DriveTeamInfo.driverPercentage));// TODO: adjust deadzones
    }

    // Driver aims to top hub or to balls
    if(leftJoy.getRawButton(1)) { //center robot on top hub (retro reflector) // Changed to button, not trigger (left front button) //TODO: test PID loop
      AimFire.centerAim("top_hub");
    }
    
    if(rightJoy.getRawButton(1)) { 
      AimFire.fire();
    }

    Climbers.climb();
    Climbers.climbrotation();
    AimFire.shooter();
    Intake.wrist();
    Intake.intake();

  }
  
  @Override
  public void teleopExit() { // Run apon exiting teleop

  }

  // Sim Code (eww)
  /**
  // @Override
  // public void simulationInit() {

  // }
  */

  /**
  // @Override
  // public void simulationPeriodic() {

  // }
  */

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