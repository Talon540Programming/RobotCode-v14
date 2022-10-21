package frc.robot.wrist.rollers.commands.control;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.wrist.rollers.WristRollersBase;

public class WristRollersControl extends CommandBase {
    private WristRollersBase rollersBase;

    protected double percentOutput;

    public WristRollersControl(WristRollersBase rollersBase) {
        this.rollersBase = rollersBase;
        addRequirements(rollersBase);
    }


    @Override
    public void execute() {
        rollersBase.setRollers(percentOutput);
    }
}
