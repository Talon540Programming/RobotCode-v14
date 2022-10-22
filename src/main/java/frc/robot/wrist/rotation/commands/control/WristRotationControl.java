package frc.robot.wrist.rotation.commands.control;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.wrist.rotation.WristRotationBase;

public class WristRotationControl extends CommandBase {
    private WristRotationBase rotationBase;

    protected double percentOutput;
    
    public WristRotationControl(WristRotationBase rotationBase) {
        this.rotationBase = rotationBase;
        addRequirements(rotationBase);
    }

    @Override
    public void execute() {
        rotationBase.setWrist(percentOutput);
    }
}