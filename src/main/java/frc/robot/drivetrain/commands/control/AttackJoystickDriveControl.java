package frc.robot.drivetrain.commands.control;

import org.talon540.control.AttackJoystick.TalonJoystick;

import frc.robot.constants.Constants;
import frc.robot.drivetrain.DrivetrainBase;

public class AttackJoystickDriveControl extends DriveControl {
    private TalonJoystick leftJoystick, rightJoystick;

    public AttackJoystickDriveControl(DrivetrainBase dBase, TalonJoystick left, TalonJoystick right) {
        super(dBase);
        this.leftJoystick = left;
        this.rightJoystick = right;
    }

    @Override
    public void execute() {
        super.kleftDrive = leftJoystick.getDeadbandY();
        super.krightDrive = rightJoystick.getDeadbandY();
        super.execute();
    }
}
