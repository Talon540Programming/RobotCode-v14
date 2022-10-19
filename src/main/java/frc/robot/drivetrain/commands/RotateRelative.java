package frc.robot.drivetrain.commands;

import frc.robot.drivetrain.DrivetrainBase;

public class RotateRelative extends RotateAbsolute {
    /**
     * Turn the drivetrain to face a relative angle from the current angle
     * @param angleOffset anlge offset
     * @param drivetrainBase
     */
    public RotateRelative(double angleOffset, DrivetrainBase drivetrainBase) {
        super(
            drivetrainBase.gyro.getRotation2d().getDegrees() + angleOffset,
            drivetrainBase
        );
    }
}
