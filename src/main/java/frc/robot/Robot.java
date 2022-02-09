// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Joystick;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private TalonFX backLeft, backRight, frontLeft, frontRight;
  private Joystick leftJoy, rightJoy;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    frontRight = new TalonFX(0);
    backRight = new TalonFX(1);
    frontLeft = new TalonFX(2);
    backLeft = new TalonFX(3);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    drive();

    double[][] shooterCalculations = getData();
    if(shooterCalculations != null) {
      System.out.println("Distance: "+shooterCalculations[0][0]+" m");
      System.out.println("Shooter Angle: "+shooterCalculations[0][1]+"°");
      System.out.println("Ideal Ball Velocity :"+shooterCalculations[0][2]+" m/s");
      System.out.println("Target Presence: "+shooterCalculations[1][0]);
      System.out.println("Horizontal Angle: "+shooterCalculations[1][1]+"°");
      System.out.println("Vertical Angle: "+shooterCalculations[1][2]+"°");
      System.out.println("Latency: "+shooterCalculations[1][3]+" ms");
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  public void drive() {
    double right = -rightJoy.getY();
    double left = -leftJoy.getY();

    if (Math.abs(right) < 0.2) {
      right = 0;
    }
    if (Math.abs(left) < 0.2) {
      left = 0;
    } 
    
    setMotors(left, right);
  }

  public void setMotors(double left, double right) {
    frontLeft.set(ControlMode.PercentOutput, left);
    backLeft.set(ControlMode.PercentOutput, left);
    frontRight.set(ControlMode.PercentOutput, -right);
    backRight.set(ControlMode.PercentOutput, -right);
  }

  public static double[][] getData() {
    double heightDifference = ((72/39.37)-(17/39.37)); //Tape height - Limelight Height
    double fixedLLANGLE = 14.7734450937; //13.0 Angle

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry tv = table.getEntry("tv");
    NetworkTableEntry tl = table.getEntry("tl");

    double[][] LimelightInfo = {{0,0,0},{0,0,0,0}};
    double targetPresent = tv.getDouble(0.0);
    double horizontalAngle = tx.getDouble(0.0);
    double verticalAngle = ty.getDouble(0.0);
    double limelightLatency = tl.getDouble(0.0);

    if(targetPresent == 1) {
      double distance = heightDifference / (Math.tan(((Math.toRadians(fixedLLANGLE)) + (Math.toRadians(verticalAngle)))));
      double angle = Math.toDegrees(Math.atan((Math.tan(Math.toRadians(-45)) * (distance)-(2 * heightDifference)) / (-distance)));
      double velocity = Math.sqrt(-1 * ((9.8 * distance * distance * (1 + (Math.pow(Math.tan(Math.toRadians(angle)), 2))) )/((2 * heightDifference)-(2 * distance * Math.tan(Math.toRadians(angle))))));
      LimelightInfo[0][0] = distance; //Horizontal distance between the hubs and the limelight
      LimelightInfo[0][1] = angle; //Optimal Shooter Angle
      LimelightInfo[0][2] = velocity; //Optimal Ball velocity
      LimelightInfo[1][0] = targetPresent; //Is the retroreflecter present?
      LimelightInfo[1][1] = horizontalAngle; //Horizontal angle between the limelight and the retroreflector
      LimelightInfo[1][2] = verticalAngle; //Vertical angle between limelight and retroreflector
      LimelightInfo[1][3] = limelightLatency; // Latency for limelight calculations

    } else {
      //spin shooter till it is detected
      return null;
    }
    
    return LimelightInfo;
  }
}
