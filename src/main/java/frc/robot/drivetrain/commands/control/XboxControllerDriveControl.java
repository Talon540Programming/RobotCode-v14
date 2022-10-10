package frc.robot.drivetrain.commands.control;

import org.talon540.control.XboxController.TalonXboxController;

import frc.robot.constants.Constants.DriveTeamInfo;
import frc.robot.drivetrain.DrivetrainBase;

public class XboxControllerDriveControl extends DriveControl {
    private TalonXboxController controller;

    public XboxControllerDriveControl(DrivetrainBase dBase, TalonXboxController cTalonXboxController) {
        super(dBase);
        this.controller = cTalonXboxController;
        // Override inherited value in case it was changed since it's initalization
        this.controller.deadband = DriveTeamInfo.XboxControllerDeadband;
    }

    @Override
    public void periodic() {
        super.kleftDrive = controller.getLeftDeadbandY();
        super.krightDrive = controller.getRightDeadbandY();
    }
}
