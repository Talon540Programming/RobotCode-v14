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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.util.Color;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String startLeft = "Left Start";
  private static final String startMiddle = "Middle Start";
  private static final String startRight = "Right Start";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  
  private TalonFX frontLeft, backLeft, frontRight, backRight;
  private TalonSRX wrist, intake, shootLeft, shootRight, index1, index2, winch, arm, level1, level2, level3, control; // Motors, change names later for readability
  private XboxController xbox; // Drive Station Xbox Controller
  private Joystick leftJoy, rightJoy; // Drive Station Joysticks
  private double left, right, hooker, wench, take, watch; // Used to set motor speeds
  private double dist, pulse, dist2, pulse2, angle, distWheel, distShooter; // Sensor values
  //private Color detectedColor; // Currently detected color
  private int counter; // Autonomous counter
  private Encoder enc1, enc2, enc3, enc4, enc5;
  private ADXRS450_Gyro gyro;
  //private LiDARSensor liDAR;
  //private ColorSensorV3 cs;
  private boolean height;

  // Camera Tracking Controls
  private double lockedAngle;
  private boolean isLocked = false;

  private String fms; // Field Management System message for Position Control

  // Unknown Constants
  private static final double DRIVE_CONSTANT = -0.8;
  private static final double WHEEL_CIRCUMFERENCE = 100.53;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("Left Start", startLeft);
    m_chooser.addOption("Middle Start", startMiddle);
    m_chooser.addOption("Right Start", startRight);
    SmartDashboard.putData("Auto choices", m_chooser);

    //Drivetrain motors
    frontRight = new TalonFX(0);
    backRight = new TalonFX(1);
    frontLeft = new TalonFX(2);
    backLeft = new TalonFX(3);

    //Shooter motors
    shootLeft = new TalonSRX(4);
    shootRight = new TalonSRX(5);
    
    //Intake
    wrist = new TalonSRX(6);
    intake = new TalonSRX(7);
    
    //indexer
    index1 = new TalonSRX(8);
    index2 = new TalonSRX(9);
    
    //Climb
    arm = new TalonSRX(10);
    winch = new TalonSRX(11);
    
    //Leveling
    level1 = new TalonSRX(12);
    level2 = new TalonSRX(13);
    level3 = new TalonSRX(14);
    
    //Contrl Panel
    control = new TalonSRX(15);

    // Gyro initialization
    gyro = new ADXRS450_Gyro();
    
    //Drivetrain enc1 TODO: setMaxPeriod, setMinRate, setDistancePulse, setSamplesToAverage
    enc1 = new Encoder(2, 3, false);
    
    //Rotation Control Encoder
    enc2 = new Encoder(4, 5, false);
    
    //Shooter Encoder
    enc3 = new Encoder(6, 7, false);

    leftJoy = new Joystick(0);
    rightJoy = new Joystick(1);
    xbox = new XboxController(2);
    
    // TODO: Get ports for censors
    //cs = new ColorSensorV3(port); 

    lockedAngle = 0.0;
    
    fms = "";
    gyro.reset();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    angle = gyro.getAngle();
    dist = enc1.getDistance();
    distShooter = enc3.getDistance();
      
    switch (m_autoSelected) {
      case startLeft:
        // TODO: code for starting on left hand side of field
        if(counter == 0) {
          moveForward(5);
        }
        if(counter == 1) {
          turnRight(115);
        }
        if(counter == 2){
          moveForward(20);
        }
        if(counter == 3){
          turnRight(65);
        }
        if(counter == 4){
          moveForward(5);
        }
      break;
      case startMiddle:
        // TODO: code for starting on center of field
        if(counter == 0) {
          moveForward(5);
        }
        if(counter == 1) {
          turnRight(25);
        }
        if(counter == 2) {
          shooterAuto();
        }
        if(counter == 3) {
          turnRight(135);
        }
        if(counter == 3) {
          moveForward(10);
        }
        if(counter == 4) {
          turnRight(45);
        }
        if(counter == 5) {
          moveForward(5);
        }
      break;
      case startRight:
        // TODO: code for starting on right hand side of field; to be programmed when strategy is finalized
        if(counter == 0) {
          turnLeft(10);
        }
        if(counter == 1){
          moveForward(7);
        }
        if(counter == 2) {
          turnRight(10);
        }
        if(counter == 3) {
          shooterAuto(); 
        }
        if(counter == 4) {
          turnLeft(180);
        }
        if(counter == 5) {
          moveForward(10);
        }
        if(counter == 6) {
          turnLeft(55);
        }
        if(counter == 7) {
          moveForward(5);
        }
        if(counter == 8) {
          turnRight(55);
        }
      break;
      case kDefaultAuto:
      default:
        turnLeft(45);
      break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    
    angle = gyro.getAngle();
    pulse = enc1.get();
    dist = enc1.getDistance();
    distWheel = enc2.getDistance();

    SmartDashboard.putNumber("Gyro angle: ", angle);
    SmartDashboard.putNumber("FL", frontLeft.getSelectedSensorPosition());
    SmartDashboard.putNumber("FR", frontRight.getSelectedSensorPosition());
    SmartDashboard.putNumber("BL", backLeft.getSelectedSensorPosition());
    SmartDashboard.putNumber("BR", backRight.getSelectedSensorPosition());

    fms = DriverStation.getInstance().getGameSpecificMessage(); // Gets the color for Position Control

    drive();
    shooter();
    climb();
    wrist();
    intake();
    cameraTracc();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  /**
   * This method takes inputs from joysticks and translates it to motor speeds for the drive train
   */
  public void drive() {
    right = -rightJoy.getY(); // Joysticks return inverted inputs (because they are made for flight simulators)
    left = -leftJoy.getY();

    if (Math.abs(right) < 0.3) { // Deadzone for joystick
      right = 0;
    }
    if (Math.abs(left) < 0.3) {
      left = 0;
    }
    setMotors(left, right);
  }

  /**
   * The shooter method for auto mode
   */
  public void shooterAuto() {
    boolean loop = true;
    while (loop) {
      distShooter = enc3.getDistance();
      if (distShooter >= 4) { // find perfect distance for the five balls shot
        shootLeft.set(ControlMode.PercentOutput, 0);
        shootRight.set(ControlMode.PercentOutput, 0);
        enc3.reset();
        counter++;
        loop = false;
      }
      else {
        shootLeft.set(ControlMode.PercentOutput, 1);
        shootRight.set(ControlMode.PercentOutput, 1);
      }
    } 
  }
  
  public void climb() {
    if (xbox.getAButton() == true) { // A Button
      hooker = 0.5;
    }
    else if (xbox.getBButton() == true) { // B Button
      hooker = -0.75;
    }
    else {
      hooker = 0;
    }
      
    if (xbox.getTriggerAxis(Hand.kRight) > 0.7) { // Right Trigger
      wench = 1;
    }
    else if (xbox.getBumper(Hand.kRight) == true) {  // Right Bumper
      wench = -1;
    }
    else {
      wench = 0; 
    }
    arm.set(ControlMode.PercentOutput, hooker);
    winch.set(ControlMode.PercentOutput, wench);
  }
  
  /**
   * The method for shooting in teleop
   */
  public void shooter() {
    //Uses X button on the Xbox Controller
    if(xbox.getRawButton(3) == true) {
        shootLeft.set(ControlMode.PercentOutput, 1);
        shootRight.set(ControlMode.PercentOutput, -1);
    }
    else {
      shootLeft.set(ControlMode.PercentOutput, 0);
      shootRight.set(ControlMode.PercentOutput, 0);
    }
  }
  
   /**
   * Used to change the position of the wrist
   */
  public void wrist() {
    // Goes up
		if (xbox.getRawAxis(2) > 0.7) { // LT
			height = true;
			wrist.set(ControlMode.PercentOutput,xbox.getRawAxis(2));
		}
		// Goes down
		else if (xbox.getRawButton(5) == true) { // LB
			height = false;
			wrist.set(ControlMode.PercentOutput,-.25);
		}
		// Maintains height
		else if (height) {
			wrist.set(ControlMode.PercentOutput, .15); //change if have to
		}
		// Stops motors if not needed
		else {
			wrist.set(ControlMode.PercentOutput, 0);
		}
  }
  
  /**
   * Rotation Control (Stage 2)
   */
  public void rotation() {
    // TODO: code for rotational control also need to bind it to a button
    for (int x = 0; x < 28; x++) { // goes through 28 colors; 3.5 rotation
        spinWheel();
    }
    
  }

  /**
   * Position Control (Stage 3)
   */
  public void position() {
      //TODO: Need to bind to a button
    if (fms.length() > 0) {
      int spin = rotateCalc(); // Finds the amount of colors to spin
      for (int i = 0; i < spin; i++) {
        spinWheel();
      }
    }
  }
  
  /**
   * The method used for spinning the wheel one color distance on the wheel
   */ 
  public void spinWheel() {
    boolean loop = true; // Loop replicates auto mode which this method is mimicking
    while(loop == true) {
      distWheel = enc2.getDistance();
      if (distWheel >= 2.67) { // 2.67 is the distance of one color on the wheel
        control.set(ControlMode.PercentOutput, 0);
        enc2.reset();
        loop = false;
      }
      else {
        control.set(ControlMode.PercentOutput, 0.3);
      }
    }
  }
  
  private void tracc(double Angle){
    if (Angle < -3.1) {
      //turnRight(-Angle);
      setMotors(-0.15, 0.15);
    } else if (Angle > 3.1) {
      //turnLeft(Angle);
      setMotors(0.15, -0.15);
    } //else {
      //moveForward(12);
    //}
}

  private void cameraTracc() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry tv = table.getEntry("tv");
    NetworkTableEntry ty = table.getEntry("ty");
    //NetworkTableEntry ta = table.getEntry("ta");

    
    //read values periodically
    double v = tv.getDouble(0.0);
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    //double area = ta.getDouble(0.0);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    double targetHeight = 77;
    double cameraHeight = 6;
    double Ldistance = (targetHeight-cameraHeight)/(Math.tan(((35.0+y)*Math.PI)/180));
    SmartDashboard.putNumber("Distance", Ldistance);
 
    if (rightJoy.getRawButton(1) && v == 1) {
      tracc(x);
    }

    /*
    if (rightJoy.getRawButton(1) && v == 1) {
      if (!isLocked) {
        isLocked = true;
        lockedAngle = x;
      }
      tracc(lockedAngle);
    }

    if(isLocked && !rightJoy.getRawButton(1)) {
      isLocked = false;
      lockedAngle = 0;
    }
    */
  }

  /**
   * The method used to calculate the amount of times needed to rotate the wheel for position control
   */ 
  public int rotateCalc() { // for position control; if used
    int timesNeedRotate;
    String find = null;
    String colorOrder = "RGBY";
    /*
    if (findColor() != null) {
      int sensorPosition = colorOrder.indexOf(findColor()); 
      int fmsPosition = colorOrder.indexOf(fms);
      if(sensorPosition >= fmsPosition){
        find = colorOrder.substring(fmsPosition, sensorPosition+1);
      }
      else if(sensorPosition < fmsPosition){
        find = colorOrder.substring(sensorPosition, fmsPosition+1);
      }
      timesNeedRotate = find.length();
      return timesNeedRotate - 2; // -2 because the place the robot is detecting will be different from the sensor the game will detect
    }
    else {
      return 0; // oopsie
    }*/
    return 0; // TODO: temporary to make this work without color sensor
  }

  /**
   * Takes an input from the ColorSensorV3 and converts it into a string to use
   * @return First letter of the color detected
   */
  /*
  public String findColor() {
    detectedColor = cs.getColor();
    if (detectedColor.equals(Color.kRed)) {
      return "R";
    }
    else if (detectedColor.equals(Color.kGreen)) {
      return "G";
    }
    else if (detectedColor.equals(Color.kBlue)) {
      return "B";
    }
    else if (detectedColor.equals(Color.kYellow)) {
      return "Y";
    }
    else {
      return null;
    }
  }
  */

  /**
   * This function sets the speed of the motors based on the parameters
   * @param l The speed of the left motors
   * @param r The speed of the right motors
   */
  public void setMotors(double l, double r) {  
    frontLeft.set(ControlMode.PercentOutput, l);
    backLeft.set(ControlMode.PercentOutput, l);
    frontRight.set(ControlMode.PercentOutput, -r);
    backRight.set(ControlMode.PercentOutput, -r);
  }
  
  /**
   * Get the proportion of moter speed based on the distance to the target value.
   * Used in auto.
   * 
   * @param target
   *            the target value
   * @param currentEnc
   *            the current encoder value
   * @return the proportion of the motor speed constant set
   */
  public static double prop(double target, double currentEnc){
    if (((target - currentEnc) / target) > 0.8) {
      return -0.4;
    }
    else if (((target - currentEnc) / target ) < 0.4) {
      return 0.3;
    }
    else{
      return (((target - currentEnc) / target) * DRIVE_CONSTANT);
    }
  }
  
  /**
   * Gets the proportion to be used with motor speed during gyro turns. Used in
   * auto.
   * Target always positive
   * 
   * @param target
   *            the target value
   * @param currentGyro
   *            the current gyro value
   * @return the proportion of the motor speed constant to set
   */ 
  public static double propGyro(double target, double currentGyro) {
    if ((1 - (currentGyro / target)) <= 0.5) {
      return 1 - (currentGyro / target);
    }
    return 0.5;
  }

  /**
   * Turns the robot to the right a specified amount. Used in auto.
   * 
   * @param targetAngle
   *            the angle to turn to (degrees)
   */ 
  private void turnRight(double targetAngle) {
    if (angle > targetAngle) {
      setMotors(0,0);
      enc1.reset();
      counter++;
    }
    else {
      setMotors(-propGyro(targetAngle, angle), propGyro(targetAngle, angle));
    }
  }

  /**
   * Turns the robot to the left a specified amount. Used in auto.
   * 
   * @param targetAngle
   *            the angle to turn to (degrees)
   */ 
  private void turnLeft(double targetAngle) {
    targetAngle = -targetAngle;
    if (angle < targetAngle) {
      setMotors(0,0);
      enc1.reset();
      counter++;
    }
    else {
      setMotors(propGyro(targetAngle, angle), -propGyro(targetAngle, angle));
    }
  }

  /**
   * Moves the robot forward in autonomous a specified amount
   * 
   * @param targetDistance
   *        the distance to travel feet
   */ 
  private void moveForward(double targetDistance) {
    if (dist >= targetDistance) {
      setMotors(0, 0);
      enc1.reset();
      gyro.reset();
      counter++;
    }
    else {
      setMotors(prop(targetDistance, dist), prop(targetDistance, dist));
    }
  }

  /**
   * Operates the intake motors using the y-axis of the right joystick
   */
  private void intake() {
	  // Ignores if value is too small
	  if (Math.abs(xbox.getY(Hand.kRight)) < 0.2) {
		  take = 0;
	  }
	  else {
	    take = xbox.getY(Hand.kRight);
	  }
	  intake.set(ControlMode.PercentOutput, take);
  }
}
