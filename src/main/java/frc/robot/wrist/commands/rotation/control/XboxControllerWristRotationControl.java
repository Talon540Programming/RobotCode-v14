package frc.robot.wrist.commands.rotation.control;

import org.talon540.control.XboxController.TalonXboxController;

import frc.robot.wrist.WristRotationBase;

public class XboxControllerWristRotationControl extends WristRotationControl {
    private TalonXboxController controller;

    public XboxControllerWristRotationControl(WristRotationBase rotationBase, TalonXboxController controller) {
        super(rotationBase);

        this.controller = controller;
    }

    @Override
    public void execute() {
        super.percentOutput = controller.buttons.X.get() ? -0.15 : controller.buttons.B.get() ? 0.15 : 0;
        super.execute();
    }
}
