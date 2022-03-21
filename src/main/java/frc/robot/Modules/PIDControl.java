package frc.robot.Modules;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Modules.Mechanisms.VisionSystems;

public class PIDControl {
    public static PIDController drivePID;
    public static PIDController flywheelPID;
    public static PIDController climbRotationPID;

    public static void init() {
        /** Drivetrain PID Controller */
        drivePID = new PIDController(RobotInformation.PID_Values.drivetrain.kP, RobotInformation.PID_Values.drivetrain.kI, RobotInformation.PID_Values.drivetrain.kD);
        
        /** Flywheel PID Controller */
        flywheelPID = new PIDController(RobotInformation.PID_Values.flywheel.kP, RobotInformation.PID_Values.flywheel.kP, RobotInformation.PID_Values.flywheel.kP);

        /** Climb Rotation PID Controller */
        climbRotationPID = new PIDController(RobotInformation.PID_Values.climbRotation.kP, RobotInformation.PID_Values.climbRotation.kI, RobotInformation.PID_Values.climbRotation.kD);


    }


    // Testing Stuff

    public static void driveDistanceTesting(double desiredDistance) {
        double distance = VisionSystems.Limelight.getDistanceFromHubStack();
        double output = PIDControl.drivePID.calculate(distance, desiredDistance);

        SmartDashboard.putNumber("Drivetrain PID Output Value", output);
    }

    public static void flywheelVelocity(double desiredVelocity) {
        double currentVelocity = MotorControl.getCurrentVelocity(Robot.shooterFly);
        
    }

}
