package frc.robot.wrist.rollers.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.wrist.rollers.WristRollersBase;

public class RollUntilIntake extends CommandBase {
    private WristRollersBase rollerBase;

    public RollUntilIntake(WristRollersBase rollersBase) {
        this.rollerBase = rollersBase;
        addRequirements(rollersBase);
    }

    @Override
    public void initialize() {
        rollerBase.clearDataMap();
        rollerBase.setRollers(-0.50);
    }

    @Override
    public boolean isFinished() {
        return rollerBase.peakFound();
    }
}