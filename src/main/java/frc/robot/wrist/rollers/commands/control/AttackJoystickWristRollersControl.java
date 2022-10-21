package frc.robot.wrist.rollers.commands.control;

import org.talon540.control.AttackJoystick.TalonJoystick;

import frc.robot.wrist.rollers.WristRollersBase;

public class AttackJoystickWristRollersControl extends WristRollersControl {
    private TalonJoystick leftJoystick, rightJoystick;

    public AttackJoystickWristRollersControl(WristRollersBase rollersBase, TalonJoystick leftJoystick, TalonJoystick rightJoystick) {
        super(rollersBase);

        this.leftJoystick = leftJoystick;
        this.rightJoystick = rightJoystick;
    }

    @Override
    public void execute() {
        super.percentOutput = leftJoystick.buttons.TOP_LEFT.get() ? -0.75 : leftJoystick.buttons.TOP_RIGHT.get() ? 0.5 : 0;
        super.execute();
    }
}
