package frc.robot.climberz.commands.control;

import org.talon540.control.XboxController.TalonXboxController;

import frc.robot.climberz.ClimberBase;

public class XboxControllerClimberzControl extends ClimberzControl {
    private TalonXboxController controller;

    public XboxControllerClimberzControl(ClimberBase sub, TalonXboxController controller) {
        super(sub);
        this.controller = controller;
    }

    @Override
    public void execute() {
        super.extensionInput = controller.buttons.DPAD_NORTH.get() ? 1 : controller.buttons.DPAD_SOUTH.get() ? -1 : 0.0;
        super.execute();
    }

}
