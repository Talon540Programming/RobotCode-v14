package frc.robot.wrist.rollers.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.wrist.rollers.WristRollersBase;

public class StopRollers extends CommandBase {
    private WristRollersBase rollerBase;

    public StopRollers(WristRollersBase wristBase) {
        this.rollerBase = wristBase;
    }

    @Override
    public void initialize() {
        rollerBase.stopRollers();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
