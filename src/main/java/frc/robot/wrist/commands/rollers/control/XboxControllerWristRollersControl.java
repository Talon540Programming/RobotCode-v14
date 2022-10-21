package frc.robot.wrist.commands.rollers.control;

import org.talon540.control.XboxController.TalonXboxController;

import frc.robot.wrist.WristRollersBase;

public class XboxControllerWristRollersControl extends WristRollersControl {
    private TalonXboxController controller;

    public XboxControllerWristRollersControl(WristRollersBase rollersBase, TalonXboxController controller) {
        super(rollersBase);

        this.controller = controller;
    }

    @Override
    public void execute() {
        super.percentOutput = controller.buttons.A.get() ? -0.75 : controller.buttons.Y.get() ? 0.5 : 0;
        super.execute();
    }
}
