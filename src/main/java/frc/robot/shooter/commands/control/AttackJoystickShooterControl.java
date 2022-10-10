package frc.robot.shooter.commands.control;

import org.talon540.control.AttackJoystick.TalonJoystick;

import frc.robot.shooter.ShooterBase;

public class AttackJoystickShooterControl extends ShooterControl {
    private TalonJoystick leftJoystick, rightJoystick;

    public AttackJoystickShooterControl(ShooterBase shooter, TalonJoystick leftJoystick, TalonJoystick rightJoystick) {
        super(shooter);
        this.leftJoystick = leftJoystick;
        this.rightJoystick = rightJoystick;
    }

    @Override
    public void periodic() {
        super.outputPercent = rightJoystick.getTrigger() ? 1 : 0;
    }
}
