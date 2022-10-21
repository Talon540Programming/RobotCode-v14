package frc.robot.wrist.commands.rollers;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.wrist.WristRollersBase;

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
