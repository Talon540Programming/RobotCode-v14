package frc.robot.drivetrain.commands;

import org.talon540.control.TalonXboxController;

import frc.robot.Constants.DriveTeamInfo;
import frc.robot.drivetrain.DrivetrainBase;

public class XboxControllerDrive extends TankDriveCommand {
    private TalonXboxController controller;

    public XboxControllerDrive(DrivetrainBase dBase, TalonXboxController cTalonXboxController) {
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
