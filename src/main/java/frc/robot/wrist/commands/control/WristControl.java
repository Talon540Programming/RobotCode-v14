package frc.robot.wrist.commands.control;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.wrist.WristBase;

public abstract class WristControl extends CommandBase {
    private WristBase wristSubsystem;
    protected double rotationPercent, rollersPercent;

    public WristControl(WristBase wristSubsystem) {
        this.wristSubsystem = wristSubsystem;
        addRequirements(wristSubsystem);
    }

    @Override
    public void execute() {
        periodic();
        this.wristSubsystem.setRollers(rollersPercent);
        this.wristSubsystem.setWrist(rotationPercent);
    }

    public abstract void periodic();
}
