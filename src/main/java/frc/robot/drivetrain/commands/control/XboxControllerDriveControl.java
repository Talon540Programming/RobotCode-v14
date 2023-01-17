package frc.robot.drivetrain.commands.control;

import org.talon540.control.XboxController.TalonXboxController;

import frc.robot.constants.Constants;
import frc.robot.drivetrain.DrivetrainBase;

public class XboxControllerDriveControl extends DriveControl {
    private TalonXboxController controller;

    public XboxControllerDriveControl(DrivetrainBase dBase, TalonXboxController cTalonXboxController) {
        super(dBase);
        this.controller = cTalonXboxController;
    }

    @Override
    public void execute() {
        super.kleftDrive = controller.getLeftDeadbandY();
        super.krightDrive = controller.getRightDeadbandY();
        super.execute();
    }
}
