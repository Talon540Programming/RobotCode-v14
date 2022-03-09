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
import frc.robot.Modules.AimFire;
import frc.robot.Modules.BallTracking;
import frc.robot.Modules.Climbers;
import frc.robot.Modules.Flywheel;
import frc.robot.Modules.Intake;
import frc.robot.Modules.RobotInformation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix.motorcontrol.ControlMode;
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
  //private TalonSRX hood; // 775s or BAG motors

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
    // STAGING FLAGS
    stage1 = false; // Auto starts in stage 1 and turns into stage 2
    ready = false; // Not ready to shoot by default
    ready2 = false;
    counter = 0;
    counter2 = 0;

    // CONTROLLER PORTS
    controller = new XboxController(2);
    leftJoy = new Joystick(0);
    rightJoy = new Joystick(1);

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

    // MOTORS
    //Drivetrain motors and configuration
    rightMaster = new WPI_TalonFX(RobotInformation.DRIVETRAIN_FRONTRIGHT);
    rightSlave = new WPI_TalonFX(RobotInformation.DRIVETRAIN_BACKRIGHT);
    rightSlave.follow(rightMaster);
    leftMaster = new WPI_TalonFX(RobotInformation.DRIVETRAIN_FRONTLEFT);
    leftSlave = new WPI_TalonFX(RobotInformation.DRIVETRAIN_BACKLEFT);
    leftSlave.follow(leftMaster);
    // Set integrated sensor position to 0 for encoder use
    // leftMaster.getSensorCollection().setIntegratedSensorPosition(0, 10);
    // rightMaster.getSensorCollection().setIntegratedSensorPosition(0, 10);

    //Climb Motors
    //climbRotation = new WPI_TalonFX(RobotInformation.CLIMBROTATION);
    climbExtension = new WPI_TalonFX(RobotInformation.CLIMBEXTENSION);

    //Intake Motors
    wrist = new WPI_TalonFX(RobotInformation.INTAKE_WRIST);
    rollers = new TalonSRX(RobotInformation.INTAKE_ROLLERS);

    //Hood Motor
    //hood = new TalonSRX(RobotInformation.SHOOTER_HOOD);

    //Shooter Motors
    shooterFly = new WPI_TalonFX(RobotInformation.SHOOTER_FLY); 
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
    
    // Used for tank and arcade drive respectively
    drive = new DifferentialDrive(leftMaster, rightMaster);

    // TODO- make this choose from sendable chooser to set alliance color
    Limelight.init();
    BallTracking.maininit();
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
      double rpm = Flywheel.getRPM(targetvelocity, 1);
      shooterFly.set(ControlMode.Velocity, rpm);
    }

    // if (controller.getXButton()) { // actuates hood to the proper angle
    //   if ((gyro.getRoll() > targetangle + 5) && !upperShooterLimit.get()) { // TODO: Adjust degrees of freedom +- 5
    //     hood.set(ControlMode.PercentOutput, 0.1);
    //     ready = false;
    //   }
    //   else if ((gyro.getRoll() < targetangle-5) && !lowerShooterLimit.get()) {
    //     hood.set(ControlMode.PercentOutput, -0.1);
    //     ready = false;
    //   }
    // }
  }

  @Override
  public void autonomousInit() {
    // m_autoSelected = m_chooser.getSelected();
    // System.out.println("Auto selected: " + m_autoSelected);
  }

  @Override
  public void autonomousPeriodic() { //Two ball auto in theory
    BallTracking.autoinit();
    // // Display information relayed by Limelight and RPM information for testing
    double[][] shooterCalculations = Limelight.getLimelightData();
    Limelight.updateSmartDashboard();

    SmartDashboard.putNumber("Flywheel RPM: ", shooterFly.getSelectedSensorVelocity()/4 * 2048);
    //SmartDashboard.putNumber("Current Angle", gyro.getRoll());

    //Taxi Code (Front Bumper needs to fully cross the tarmac)
    if(Limelight.hubPresent() && (shooterCalculations[0][0]<(RobotInformation.botlengthMeters+RobotInformation.tarmacLengthMeters+0.1))) { //If the top hub is present and we are less than 2.3 meters away drive backwards
      drive.tankDrive(-0.1, -0.1);
    } else if(Limelight.hubPresent() && (shooterCalculations[0][0]>(RobotInformation.botlengthMeters+RobotInformation.tarmacLengthMeters+0.6))) { //If we overshoot the target
      drive.tankDrive(0.1, 0.1);
    } else {
      drive.tankDrive(0,0);
    }
    // if ((shooterCalcuations[0][0] < 2.3) && !ready) { // Counter
    //   drive.tankDrive(-0.1, -0.1);
    // } 
    // else if ((shooterCalculations[0][0] >= 2.3) && !ready) {
    //   ready = true;
    // }
    // if (ready && !ready2) {
    //   drive.tankDrive(0,0);
      // if (stage1) {
      //   shooterFly.set(ControlMode.PercentOutput, 0.7); 
      //   counter++;
      //   if ((counter < 100)) {
      //     stage1 = false;
      //   }
      // }
      // if (counter >= 100) {
      //   rollers.set(ControlMode.PercentOutput, 0.4);
      //   counter2++;
      // }
      // if (counter2  >= 100) {
      //   ready2 = true;
      // }
    // }
    // if (ready2) {
    //   shooterFly.set(ControlMode.PercentOutput, 0);
    //   rollers.set(ControlMode.PercentOutput, 0);
    // }

    // // Resets hood to position 0 and uses that as the 0-angle.
    // // if (!lowerShooterLimit.get()) {
    // //   hood.set(ControlMode.PercentOutput, -0.1);
    // // }
    // gyro.reset(); //sets gyro value to 0

    // if ((Math.abs(shooterCalculations[0][0]) < 1.5)) { 
    //   drive.tankDrive(-0.8, -0.8); // backs up until 1.5 meters away
    // }
    // if ((Math.abs(shooterCalculations[0][0]) >= 1.5) && (Math.abs(shooterCalculations[1][0]) > 1)) { // TODO: adjust to reduce bouncing
    //   centerAim("top_hub"); // fires a shot into the hub
    // }
    // if (Math.abs(shooterCalculations[1][0]) < 1 && (Math.abs(shooterCalculations[0][0]) >= 1.5) && stage1) {
    //   if (counter < 1000) { // TODO: adjust based on how long it takes to shoot a ball
    //     fire(); // shoots ball-> changes stage1 to false after having shot a ball
    //     counter++;
    //   }
    //   else if (counter >= 1000) {
    //     stage1 = false;
    //   }
    // }
    // if (!stage1 && !ballSeen()) { // Locates ball if it can't see after shooting one
    //   centerAim("ball_tracking");
    //   rollers.set(ControlMode.PercentOutput, 0);
    // }
    // if (!stage1 && ballSeen()) { // Drives forward if it sees the ball
    //   drive.tankDrive(0.8, 0.8);
    //   rollers.set(ControlMode.PercentOutput, 1);
    // }
  }
  
  @Override
  public void teleopPeriodic() {
    double[][] shooterCalculations = Limelight.getLimelightData();
    Limelight.updateSmartDashboard();

    SmartDashboard.putNumber("Flywheel RPM: ", shooterFly.getSelectedSensorVelocity()/4 * 2048);
    // SmartDashboard.putNumber("Current Angle", gyro.getRoll());

    // DRIVE CALL
    if ((Math.abs(rightJoy.getY()) > 0.2) || Math.abs(leftJoy.getY()) > 0.2) {
      drive.tankDrive(-rightJoy.getY() * RobotInformation.driverPercentage, leftJoy.getY() *RobotInformation.driverPercentage);// TODO: adjust deadzones
    }

    // Driver aims to top hub or to balls
    // if(leftJoy.getTrigger()) { //center robot on top hub (retro reflector)
    //   centerAim("top_hub");
    // }
    if(leftJoy.getRawButton(1)) { //center robot on top hub (retro reflector) // Changed to button, not trigger (left front button)
      AimFire.centerAim("top_hub");
    }
    // if(rightJoy.getTrigger()) { //center robot on ball from Raspberry Pi Data
    //   centerAim("ball_tracking");
    // }
    
    //climbrotation();
    Climbers.climb();
    // if (controller.getAButton()) fire();
    // else ready = false;
    AimFire.shooter();
    //hood();
    Intake.wrist();
    Intake.intake();
  }
}