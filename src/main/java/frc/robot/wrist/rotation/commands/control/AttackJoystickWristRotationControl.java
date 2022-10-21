package frc.robot.wrist.rotation.commands.control;

import org.talon540.control.AttackJoystick.TalonJoystick;

import frc.robot.wrist.rotation.WristRotationBase;

public class AttackJoystickWristRotationControl extends WristRotationControl {
    private TalonJoystick leftJoystick, rightJoystick;

    public AttackJoystickWristRotationControl(WristRotationBase rotationBase, TalonJoystick leftJoysitck, TalonJoystick rightJoystick) {
        super(rotationBase);

        this.leftJoystick = leftJoysitck;
        this.rightJoystick = rightJoystick;
    }

    @Override
    public void execute() {
        super.percentOutput = rightJoystick.buttons.TOP_LEFT.get() ? -0.15 : rightJoystick.buttons.TOP_RIGHT.get() ? 0.15 : 0;
        super.execute();
    }
}
