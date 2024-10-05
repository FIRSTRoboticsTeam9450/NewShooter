package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.LauncherMode;

public class ResetClimbCommand extends Command {
    
    ClimbSubsystem climb = ClimbSubsystem.getInstance();

    @Override
    public void initialize() {
        Launcher.currentMode = LauncherMode.SPEAKER;
        climb.runDownAndReset();
    }

    @Override
    public boolean isFinished() {
        return !climb.isResetting();
    }

}
