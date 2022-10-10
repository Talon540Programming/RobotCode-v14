package frc.robot.climberz.commands;

import org.talon540.control.XboxController.TalonXboxController;

import frc.robot.climberz.ClimberBase;

public class XboxControllerClimberz extends ClimberzControl {
    private TalonXboxController controller;

    public XboxControllerClimberz(ClimberBase sub, TalonXboxController controller) {
        super(sub);
        this.controller = controller;
    }

    @Override
    public void periodic() {
        super.extensionInput = controller.buttons.DPAD_NORTH.get() ? 0.3 : controller.buttons.DPAD_SOUTH.get() ? -0.3 : 0.0;
        super.rotationInput = controller.buttons.DPAD_EAST.get() ? 0.3 : controller.buttons.DPAD_WEST.get() ? -0.3 : 0.0;
    }
}
