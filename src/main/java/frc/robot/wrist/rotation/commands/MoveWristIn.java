package frc.robot.wrist.rotation.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.wrist.rotation.WristRotationBase;

public class MoveWristIn extends CommandBase {
    private WristRotationBase rotationBase;

    public MoveWristIn(WristRotationBase rotationBase) {
        this.rotationBase = rotationBase;
        addRequirements(rotationBase);
    }

    @Override
    public void initialize() {
        this.rotationBase.setWristRaw(0.15);
    }

    @Override
    public boolean isFinished() {
        return rotationBase.underResistance();
    }
}
