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
import com.kauailabs.navx.frc.AHRS;


public class Robot extends TimedRobot {
  private int counter = 0;
  private static final double WHEEL_DIAMETER = 6; // inches
  //NEED TO FIGURE OUT ENCODER MODEL
  private static final double cpr = 360; //if am-3132
  //private static final double cpr = 7/4; //if am-2861a
  // private static final double cpr = 5; //if am-3314a
  // private static final double cpr = 1024; //if am-3445
  // private static final double cpr = 64; //if am-4027

  private static final double drive_dpp = (Math.PI*WHEEL_DIAMETER/cpr); // FOR DRIVETRAIN WHEELS ONLY

  private boolean ready;
  
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private WPI_TalonFX leftSlave, rightSlave, leftMaster, rightMaster, climbRotation, climbExtension, shooterFly, wrist;
  private TalonSRX rollers, hood;
  private Joystick leftJoy, rightJoy;
  private XboxController controller = new XboxController(2);

  private AHRS gyro;
  
  private DifferentialDrive drive;
  private double nonZeroLimelightHorAng;

  private Encoder rightEncoder, leftEncoder;
  private DigitalInput shooterLimit;
  private DigitalInput intakeLimit;
  
  //auto variables
  private boolean stage1; 

  //private ADXRS450_Gyro gyro;
  //private LiDARSensor liDAR;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    stage1 = true;
    ready = false;

    gyro = new AHRS(SerialPort.Port.kMXP); //SUBJECT TO CHANGE FROM ELECTRICAL COULD BE SOURCE OF ERROR

    //Drivetrain motors and configuration
    rightMaster = new WPI_TalonFX(RobotMap.DRIVETRAIN_FRONTRIGHT);
    rightSlave = new WPI_TalonFX(RobotMap.DRIVETRAIN_BACKRIGHT);
    rightSlave.follow(rightMaster);
    leftMaster = new WPI_TalonFX(RobotMap.DRIVETRAIN_FRONTLEFT);
    leftSlave = new WPI_TalonFX(RobotMap.DRIVETRAIN_BACKLEFT);
    leftSlave.follow(leftMaster);

    // Used for tank and arcade drive respectively
    drive = new DifferentialDrive(leftMaster, rightMaster);

    // Default integrated sensors
    leftMaster.getSensorCollection().setIntegratedSensorPosition(0, 10);
    rightMaster.getSensorCollection().setIntegratedSensorPosition(0, 10);


    // ALL Ports subject to change from Electrical
    leftEncoder = new Encoder(0, 1);
    leftEncoder.setDistancePerPulse(drive_dpp);
    rightEncoder = new Encoder(2, 3);
    rightEncoder.setDistancePerPulse(drive_dpp);

    // Control Initializations
    leftJoy = new Joystick(0);
    rightJoy = new Joystick(1);

    //Climb Motors
    climbRotation = new WPI_TalonFX(RobotMap.CLIMBROTATION);
    climbExtension = new WPI_TalonFX(RobotMap.CLIMBEXTENSION);

    //Shooter Motors
    shooterFly = new WPI_TalonFX(RobotMap.SHOOTER_FLY);
    shooterFly.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 1);
    shooterFly.configNominalOutputForward(0, 1);
    shooterFly.configNominalOutputReverse(0, 1);
    shooterFly.configPeakOutputForward(1, 1);
    shooterFly.configPeakOutputReverse(-1, 1);
    shooterFly.setInverted(false);
    shooterFly.setSensorPhase(false);
    // config P,I,D,F values- start by doubling F, then P, then D, then I (middle values) then increase over time
    shooterFly.config_kF(0, 0.007, 1);
    shooterFly.config_kP(0, 0.8192, 1);
    shooterFly.config_kI(0, 0.0008, 1);
    shooterFly.config_kD(0, 0.0256, 1);

    //Intake Motors
    wrist = new WPI_TalonFX(RobotMap.INTAKE_WRIST);
    rollers = new TalonSRX(RobotMap.INTAKE_ROLLERS);

    //Hood Motor
    hood = new TalonSRX(RobotMap.SHOOTER_HOOD);

    if (!shooterLimit.get()) {
      hood.set(ControlMode.PercentOutput, -0.1);
    }
    gyro.reset();

    // Display information
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0); //Sets the pipeline to 0
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0); //Sets the Limelight as a Vision Proccesor
    NetworkTableInstance.getDefault().getTable("TalonPi").getEntry("alliance").setString("red");
  }
  

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    double[][] shooterCalculations = getLimelightData();
    if(shooterCalculations != null) {
      SmartDashboard.putNumber("Distance: ",shooterCalculations[0][0]);
      SmartDashboard.putNumber("Shooter Angle: ",shooterCalculations[0][1]);
      SmartDashboard.putNumber("Ideal Ball Velocity :",shooterCalculations[0][2]);
      SmartDashboard.putNumber("Limelight H-Angle: ",shooterCalculations[1][0]);
      SmartDashboard.putNumber("Limelight V-Angle: ",shooterCalculations[1][1]);
      SmartDashboard.putNumber("Limelight Latency: ",shooterCalculations[1][2]);
      SmartDashboard.putNumber("Flywheel RPM: ", shooterFly.getSelectedSensorVelocity()/4 * 2048);
    }

    if ((Math.abs(shooterCalculations[0][0]) < 1.5)) { 
      drive.tankDrive(-0.8, -0.8); // backs up until 1.5 meters away
    }
    if ((Math.abs(shooterCalculations[0][0]) >= 1.5) && (Math.abs(shooterCalculations[1][0]) > 1)) { //adjust to reduce bouncing
      centerAim("top_hub"); // fires a shot into the hub
    }
    if (Math.abs(shooterCalculations[1][0]) < 1 && (Math.abs(shooterCalculations[0][0]) >= 1.5) && stage1) {
      if (counter < 1000) { // adjust based on how long it takes to shoot a ball
        autoFire(); // shoots ball-> changes stage1 to false after having shot a ball
        counter++;
      }
      else if (counter >= 1000) {
        stage1 = false;
      }
    }
    if (!stage1 && !ballSeen()) { // Locates ball if it can't see after shooting one
      centerAim("ball_tracking");
      rollers.set(ControlMode.PercentOutput, 0);
    }
    if (!stage1 && ballSeen()) { // Drives forward if it sees the ball
      drive.tankDrive(0.8, 0.8);
      rollers.set(ControlMode.PercentOutput, 1);
    }
  }
  
  /**
   * The shooter method for auto mode
   */

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    

    //xbox.setRumble(RumbleType.kLeftRumble, 0.5);
    //xbox.setRumble(RumbleType.kRightRumble, 0.5);

    // reset button
    if (leftJoy.getTrigger()) {
      leftMaster.getSensorCollection().setIntegratedSensorPosition(0, 10);
      rightMaster.getSensorCollection().setIntegratedSensorPosition(0, 10);
    }

    if ((Math.abs(rightJoy.getY()) > 0.2) || Math.abs(leftJoy.getY()) > 0.2) drive.tankDrive(-rightJoy.getY(), -leftJoy.getY());// deadzones

    
    double[][] shooterCalculations = getLimelightData();
    if(shooterCalculations != null) {
      SmartDashboard.putNumber("Distance: ",shooterCalculations[0][0]);
      SmartDashboard.putNumber("Shooter Angle: ",shooterCalculations[0][1]);
      SmartDashboard.putNumber("Ideal Ball Velocity :",shooterCalculations[0][2]);
      SmartDashboard.putNumber("Limelight H-Angle: ",shooterCalculations[1][0]);
      SmartDashboard.putNumber("Limelight V-Angle: ",shooterCalculations[1][1]);
      SmartDashboard.putNumber("Limelight Latency: ",shooterCalculations[1][2]);
    }
    if(leftJoy.getRawButton(1)) { //center robot on top hub (retro reflector)
      centerAim("top_hub");
    }
    if(rightJoy.getRawButton(2)) { //center robot on ball from Raspberry Pi Data
      centerAim("ball_tracking");
    }

  }

  public double[][] getLimelightData() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    double heightDifference = ((72/39.37)-(17/39.37)); //13.0 (photo room)
    // double heightDifference = ((104/39.37)-(24.025/39.37)); //14.0 https://www.desmos.com/calculator/zmzfln2j6v
    double fixedLLANGLE = 14.7734450937; //13.0 Angle
    // double fixedLLANGLE = 40; //14.0 Angle https://www.desmos.com/calculator/zmzfln2j6v

    double[][] LimelightInfo = {{0,0,0},{0,0,0}};

    double horizontalAngle = table.getEntry("tx").getDouble(0);
    double verticalAngle = table.getEntry("ty").getDouble(0);
    double limelightLatency = table.getEntry("tl").getDouble(0);

    double distance = heightDifference / (Math.tan(((Math.toRadians(fixedLLANGLE)) + (Math.toRadians(verticalAngle)))));
    double angle = Math.toDegrees(Math.atan((Math.tan(Math.toRadians(-45)) * (distance)-(2 * heightDifference)) / (-distance)));
    double velocity = Math.sqrt(-1 * ((9.8 * distance * distance * (1 + (Math.pow(Math.tan(Math.toRadians(angle)), 2))) )/((2 * heightDifference)-(2 * distance * Math.tan(Math.toRadians(angle))))));

    LimelightInfo[0][0] = distance; //Horizontal distance between the hubs and the limelight
    LimelightInfo[0][1] = angle; //Optimal Shooter Angle
    LimelightInfo[0][2] = velocity; //Optimal Ball velocity
    LimelightInfo[1][0] = horizontalAngle; //Horizontal angle between the limelight and the retroreflector
    LimelightInfo[1][1] = verticalAngle; //Vertical angle between limelight and retroreflector
    LimelightInfo[1][2] = limelightLatency; // Latency for limelight calculations
    
    if(horizontalAngle != 0) {
      nonZeroLimelightHorAng = horizontalAngle;
    }

    return LimelightInfo;
  }

  public void centerAim(String target) { 
    //https://i.kym-cdn.com/entries/icons/original/000/039/393/cover2.jpg
    double deadbandAngleRange = 0.5; //close enough
    
    switch(target) {
      case "top_hub":
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0); //Sets the pipeline to 0
        getLimelightData();
        if(Math.abs(nonZeroLimelightHorAng)>deadbandAngleRange) { //Is the number within the deadband range?
          if(nonZeroLimelightHorAng>0) { //Positive
            // Turn right
            double motorSpeed = (Math.abs(nonZeroLimelightHorAng * .9)/59.6)+.05;
            motorSpeed = Math.round(motorSpeed * 100) / 100.0;
            drive.tankDrive(motorSpeed, -motorSpeed);
          }
          if(nonZeroLimelightHorAng<0) { //Negetive
            // Turn left
            double motorSpeed = (Math.abs(nonZeroLimelightHorAng * .9)/59.6)+.05;
            motorSpeed = Math.round(motorSpeed * 100) / 100.0;
            drive.tankDrive(-motorSpeed, motorSpeed);
          }
        }
        break;

      case "ball_tracking": // TODO: actually make this work- Sriman and Ayush
        NetworkTable piTable = NetworkTableInstance.getDefault().getTable("TalonPi");
        double piAngle = piTable.getEntry("ang").getDouble(0);

        break;
    }
  }

  private boolean ballSeen() {
    return true;
  } // Sriman/Ayush code this so we know if there's a ball



  //Shooter Code
  public void fire() {
    double transferPercent = 0.35; 
    double[][] limelightData = getLimelightData();
    double idealAngle = limelightData[0][1];
    double idealVelocity = limelightData[0][2];
    double idealRPM = (((idealVelocity*(1/transferPercent))/(Math.PI*0.1016))*60); // flywheel diameter in meters
    if (!intakeLimit.get()) {
      wrist.set(ControlMode.PercentOutput, 0.75);
    }
    shooterFly.set(ControlMode.Velocity, 4*idealRPM); //Gear ratio multiplier MIGHT HAVE TO MULTIPLY BY 2048- will test later
    if (gyro.getRoll() > idealAngle + 5) { // degrees of freedom +- 5
      hood.set(ControlMode.PercentOutput, 0.1);
      ready = false;
    }
    else if (gyro.getRoll() < idealAngle-5) {
      hood.set(ControlMode.PercentOutput, -0.1);
      ready = false;
    }
    else {
      ready = true;
    }
    if (ready) {
      rollers.set(ControlMode.PercentOutput, 1);
    }
    //adjust to fine tuning 
    // double OneRPM = (60*idealVelocity)/(8*Math.PI); //RPM required if 100% of speed from flywheel is transferred to the ball
    // double actualRPM = 10*OneRPM; //RPM needed bearing 10% of velocity is transferred from flywheel to ball
    // double flywheelMotor = actualRPM/the maximum rpm of the motor; //The motor output in percentage of the flywheel motor
  }

  public boolean autoFire() {
    double transferPercent = 0.35; 
    double[][] limelightData = getLimelightData();
    double idealAngle = limelightData[0][1];
    double idealVelocity = limelightData[0][2];
    double idealRPM = (((idealVelocity*(1/transferPercent))/(Math.PI*0.1016))*60); // flywheel diameter in meters
    if (!intakeLimit.get()) {
      wrist.set(ControlMode.PercentOutput, 0.75);
    }
    shooterFly.set(ControlMode.Velocity, 4*idealRPM); //Gear ratio multiplier MIGHT HAVE TO MULTIPLY BY 2048- will test later
    if (gyro.getRoll() > idealAngle + 5) { // degrees of freedom +- 5
      hood.set(ControlMode.PercentOutput, 0.1);
      ready = false;
    }
    else if (gyro.getRoll() < idealAngle-5) {
      hood.set(ControlMode.PercentOutput, -0.1);
      ready = false;
    }
    else {
      ready = true;
    }
    if (ready) {
      rollers.set(ControlMode.PercentOutput, 1);
    }
    return false;
    //adjust to fine tuning 
    // double OneRPM = (60*idealVelocity)/(8*Math.PI); //RPM required if 100% of speed from flywheel is transferred to the ball
    // double actualRPM = 10*OneRPM; //RPM needed bearing 10% of velocity is transferred from flywheel to ball
    // double flywheelMotor = actualRPM/the maximum rpm of the motor; //The motor output in percentage of the flywheel motor
  }

  
  //Spinny part of intake, no encoder
  public void intake() {
    if(Math.abs(controller.getRightY()) > 0.2) { //Right trigger spins intake from field to robot
      rollers.set(ControlMode.PercentOutput, controller.getRightTriggerAxis());
    } 
    else if(Math.abs(controller.getRightY()) < -0.2 ) { //Left trigger spins intake from entrance of robot to flywheel
      rollers.set(ControlMode.PercentOutput, -controller.getLeftTriggerAxis());
      } 
    else {
      rollers.set(ControlMode.PercentOutput, 0);
    }
  }

  //Wrist for intake, no encoder (for teleop)
  public void wrist() {
    if (controller.getBButton()) { //sets intake in position to feed ball into flywheel
      wrist.set(ControlMode.PercentOutput, 0.75);
    } else if (controller.getXButton()) { //sets intake in position to pick balls off field
        wrist.set(ControlMode.PercentOutput, -0.4);
    } else {
        wrist.set(ControlMode.PercentOutput, 0);
    }
  }

  //Shooter, will probably be replaced with autonomous shooter stuff, need encoder
  public void shooter() {
    if(controller.getRightBumper()) {
      shooterFly.set(ControlMode.PercentOutput, 1);
    } else {
      shooterFly.set(ControlMode.PercentOutput, 0);
    }
  }


  //Vertical Climb, no encoder
  public void climb() {
    if (controller.getPOV() == 0) { //Up on D pad
      climbExtension.set(ControlMode.PercentOutput, 1);
    } else if (controller.getPOV() == 180) { //Down on D pad
      climbExtension.set(ControlMode.PercentOutput, -1);
    } else {
      climbExtension.set(ControlMode.PercentOutput, 0);
    }
  }

  //Climb Rotation, no encoder
  public void climbrotation() {
    if (controller.getPOV() == 90) { // Right on the D pad
      climbRotation.set(ControlMode.PercentOutput, 1);
    }
    else if (controller.getPOV() == 270) { //Left on the D pad
      climbRotation.set(ControlMode.PercentOutput, -1);
    }
    else {
      climbRotation.set(ControlMode.PercentOutput, 0);
    }
  }
}