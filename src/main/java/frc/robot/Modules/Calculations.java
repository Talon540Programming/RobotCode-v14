package frc.robot.Modules;

import frc.robot.Modules.Mechanisms.VisionSystems;
import frc.robot.Modules.RobotInformation.FieldData.ValidTargets;

public class Calculations {
    /**
     * This function calculates the desired velocity of the flywheel for a given target
     *
     * @param target The target that the robot is shooting at.
     * @return The desired velocity of the ball.
     */
    public static double getDesiredBallVelocity(ValidTargets target) {
        switch(target) {
            case upper_hub:
                return Math.sqrt(-1 * ((9.8 * VisionSystems.Limelight.getDistanceFromHubStack() * VisionSystems.Limelight.getDistanceFromHubStack() * (1 + (Math.pow(Math.tan(Math.toRadians(RobotInformation.RobotData.RobotMeasurement.shooterHoodAngle)), 2))) )/((2 * (RobotInformation.FieldData.upperHubHeightMeters-RobotInformation.RobotData.RobotMeasurement.flywheelHeightMeters))-(2 * VisionSystems.Limelight.getDistanceFromHubStack() * Math.tan(Math.toRadians(RobotInformation.RobotData.RobotMeasurement.shooterHoodAngle))))));
            case lower_hub:
                return Math.sqrt(-1 * ((9.8 * VisionSystems.Limelight.getDistanceFromHubStack() * VisionSystems.Limelight.getDistanceFromHubStack() * (1 + (Math.pow(Math.tan(Math.toRadians(RobotInformation.RobotData.RobotMeasurement.shooterHoodAngle)), 2))) )/((2 * (RobotInformation.FieldData.lowerHubHeightMeters-RobotInformation.RobotData.RobotMeasurement.flywheelHeightMeters))-(2 * VisionSystems.Limelight.getDistanceFromHubStack() * Math.tan(Math.toRadians(RobotInformation.RobotData.RobotMeasurement.shooterHoodAngle))))));
            default:
                return 0;
        }
    }
}