package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command autonomousCommand;

    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        CameraServer.startAutomaticCapture();
        robotContainer = new RobotContainer();
        robotContainer.disableAllLights();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        if(DriverStation.isEStopped()) robotContainer.reportError();
        robotContainer.updateSmartDashboard();
    }

    @Override
    public void disabledInit() {
        robotContainer.disableFunctionalLights();
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {
        robotContainer.enableFunctionalLights();

        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        robotContainer.enableFunctionalLights();

        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}
}
