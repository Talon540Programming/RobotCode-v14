package frc.robot.climberz.commands.control;

import org.talon540.control.AttackJoystick.TalonJoystick;

import frc.robot.climberz.ClimberBase;

public class AttackJoystickClimberzControl extends ClimberzControl {
    private TalonJoystick leftJoystick, rightJoystick;

    public AttackJoystickClimberzControl(ClimberBase sub, TalonJoystick leftJoystick, TalonJoystick rightJoystick) {
        super(sub);
        this.leftJoystick = leftJoystick;
        this.rightJoystick = rightJoystick;
    }

    @Override
    public void periodic() {
        super.extensionInput = leftJoystick.buttons.TOP_MIDDLE.get() ? 0.3 : leftJoystick.buttons.TOP_BOTTOM.get() ? -0.3 : 0.0;
    }
}
