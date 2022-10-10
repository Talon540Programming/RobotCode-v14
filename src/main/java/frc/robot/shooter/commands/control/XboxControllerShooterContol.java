package frc.robot.shooter.commands.control;

import org.talon540.control.XboxController.TalonXboxController;

import frc.robot.shooter.ShooterBase;
import frc.robot.shooter.commands.ShooterControl;

public class XboxControllerShooterContol extends ShooterControl {
    private TalonXboxController controller;

    public XboxControllerShooterContol(ShooterBase shooter, TalonXboxController controller) {
        super(shooter);
        this.controller = controller;
    }

    @Override
    public void periodic() {
        super.outputPercent = controller.buttons.RIGHT_BUMPER.get() ? 1 : 0;
    }
}
