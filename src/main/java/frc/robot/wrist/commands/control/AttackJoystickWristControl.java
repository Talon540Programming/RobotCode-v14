package frc.robot.wrist.commands.control;

import org.talon540.control.AttackJoystick.TalonJoystick;

import frc.robot.wrist.WristBase;

public class AttackJoystickWristControl extends WristControl {
    private TalonJoystick leftJoystick, rightJoystick;

    public AttackJoystickWristControl(WristBase wristSub, TalonJoystick leftJoystick, TalonJoystick rightJoystick) {
        super(wristSub);

        this.leftJoystick = leftJoystick;
        this.rightJoystick = rightJoystick;
    }

    @Override
    public void execute() {
        super.rollersPercent =  leftJoystick.buttons.TOP_LEFT.get() ? -0.5 : leftJoystick.buttons.TOP_RIGHT.get() ? 0.5 : 0;
        super.rotationPercent =  rightJoystick.buttons.TOP_LEFT.get() ? -0.15 : rightJoystick.buttons.TOP_RIGHT.get() ? 0.15 : 0;
        super.execute();
    }
}
