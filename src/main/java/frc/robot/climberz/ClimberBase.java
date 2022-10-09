package frc.robot.climberz;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class ClimberBase extends SubsystemBase {
    private WPI_TalonFX climbExtension, climbRotation;

    public ClimberBase() {
        this.climbExtension = new WPI_TalonFX(Constants.RobotData.RobotPorts.CLIMBEXTENSION);
        this.climbRotation = new WPI_TalonFX(Constants.RobotData.RobotPorts.CLIMBROTATION);

        this.climbExtension.setNeutralMode(NeutralMode.Brake);
        this.climbRotation.setNeutralMode(NeutralMode.Brake);
    }
}
