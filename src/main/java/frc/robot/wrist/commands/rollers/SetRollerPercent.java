package frc.robot.wrist.commands.rollers;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.wrist.WristBase;

public class SetRollerPercent extends  CommandBase {
    private WristBase wristBase;
    private double percent;

    public SetRollerPercent(WristBase wristBase, double percentOut) {
        this.wristBase = wristBase;
        this.percent = percentOut;
        addRequirements(wristBase);
    }

    @Override
    public void initialize() {
        wristBase.setRollers(percent);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}