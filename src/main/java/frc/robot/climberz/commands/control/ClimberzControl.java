package frc.robot.climberz.commands.control;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.climberz.ClimberBase;

public abstract class ClimberzControl extends CommandBase {
    protected ClimberBase climberzSub;

    protected double extensionInput;

    ClimberzControl(ClimberBase climberzSubsystem) {
        climberzSub = climberzSubsystem;
        addRequirements(climberzSubsystem);
    }

    @Override
    public void execute() {
        climberzSub.setExtensionPercentOut(extensionInput);
    }
}
