package frc.robot.shooter.commands.control;

import org.talon540.control.XboxController.TalonXboxController;

import frc.robot.shooter.ShooterBase;

public class XboxControllerShooterContol extends ShooterControl {
    private TalonXboxController controller;

    public XboxControllerShooterContol(ShooterBase shooter, TalonXboxController controller) {
        super(shooter);
        this.controller = controller;
    }

    @Override
    public void periodic() {
        super.outputPercent = controller.buttons.RIGHT_TRIGGER.get() ? 1 : 0;
    }
}
