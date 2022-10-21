package frc.robot.wrist.commands.rollers;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.wrist.WristRollersBase;

/**
 * Set the rollers percent output
 * @implNote This commands does not stop the motor after it finishes
 */
public class SetRollers extends CommandBase {
    private WristRollersBase rollersBase;
    private double percent;

    public SetRollers(WristRollersBase rollersBase, double percentOut) {
        this.rollersBase = rollersBase;
        this.percent = percentOut;
        addRequirements(rollersBase);
    }

    @Override
    public void initialize() {
        rollersBase.setRollers(percent);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}