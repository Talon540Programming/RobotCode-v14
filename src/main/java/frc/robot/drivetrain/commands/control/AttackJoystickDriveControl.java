package frc.robot.drivetrain.commands.control;

import org.talon540.control.AttackJoystick.TalonJoystick;

import frc.robot.constants.Constants.DriveTeamInfo;
import frc.robot.drivetrain.DrivetrainBase;

public class AttackJoystickDriveControl extends DriveControl {
    private TalonJoystick leftJoystick, rightJoystick;

    public AttackJoystickDriveControl(DrivetrainBase dBase, TalonJoystick left, TalonJoystick right) {
        super(dBase);
        this.leftJoystick = left;
        this.rightJoystick = right;

        // Override inherited value in case it was changed since it's initalization
        this.leftJoystick.deadband = DriveTeamInfo.AttackJoystickDeadband;
        this.rightJoystick.deadband = DriveTeamInfo.AttackJoystickDeadband;
    }

    @Override
    public void periodic() {
        super.kleftDrive = leftJoystick.getDeadbandY();
        super.krightDrive = rightJoystick.getDeadbandY();
    }
}
