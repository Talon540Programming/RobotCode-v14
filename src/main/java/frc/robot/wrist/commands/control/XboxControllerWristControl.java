package frc.robot.wrist.commands.control;

import org.talon540.control.XboxController.TalonXboxController;

import frc.robot.wrist.WristBase;

public class XboxControllerWristControl extends WristControl {
    private TalonXboxController controller;

    public XboxControllerWristControl(WristBase wristSubsystem, TalonXboxController controller) {
        super(wristSubsystem);
        this.controller = controller;
    }

    @Override
    public void periodic() {
        super.rollersPercent =  controller.buttons.A.get() ? -0.75 : controller.buttons.Y.get() ? 1 : 0;
        super.rotationPercent =  controller.buttons.X.get() ? -0.15 : controller.buttons.B.get() ? 0.15 : 0;
    }
}
