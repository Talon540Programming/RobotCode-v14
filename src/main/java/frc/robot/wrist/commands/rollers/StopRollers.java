package frc.robot.wrist.commands.rollers;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.wrist.WristBase;

public class StopRollers extends CommandBase {
    private WristBase wristBase;

    public StopRollers(WristBase wristBase) {
        this.wristBase = wristBase;
    }

    @Override
    public void initialize() {
        wristBase.stopRollers();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
