package frc.robot.shooter.commands;

import org.talon540.vision.Limelight.LimelightVision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.shooter.ShooterBase;

public class AutoSpeedShooter extends CommandBase {

    private LimelightVision limelight;
    private ShooterBase shooterBase;

    private double targetVelocity, currentVelocity;

    public AutoSpeedShooter(ShooterBase shooterBase, LimelightVision limelightBase) {
        this.limelight = limelightBase;
        this.shooterBase = shooterBase;
    }

    @Override
    public void execute() {
        double hubHeight = Constants.FieldData.upperHubHeightMeters;
        double distanceFromHubstack = limelight.getDistanceFromTargetBase(hubHeight);
        double shooterHoodAngleTan = Math.tan(Math.toRadians(Constants.RobotData.RobotMeasurement.shooterHoodAngle));
        double flywheelHeight = Constants.RobotData.RobotMeasurement.flywheelHeightMeters;
        double targetHeightDelta = hubHeight - flywheelHeight;

        targetVelocity = Math.sqrt(-(9.8*Math.pow(distanceFromHubstack,2)*(1+Math.pow(shooterHoodAngleTan,2)))/((2*targetHeightDelta)-2*distanceFromHubstack*shooterHoodAngleTan));

        shooterBase.setFlywheelLinearVelocity(targetVelocity);
        currentVelocity = shooterBase.sensorCollection.getLinearVelocity();
    }

    @Override
    public boolean isFinished() {
        return targetVelocity * 0.95 <= currentVelocity && currentVelocity <= targetVelocity * 1.05;
    }
}
