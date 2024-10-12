package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.RobotConstants;
import frc.RobotConstants.LauncherConstants;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.LauncherMode;

/**
 * Based on the mode, it will choose speeds for the launcher then launch the note
 */
public class RunRobotMode extends Command { // RENAME PLS
    
    // Create different commands based on modes using different values
    Command amp = new SequentialCommandGroup(
        new SetLauncherSpeedCommand(LauncherConstants.upperVelocityAmp, LauncherConstants.lowerVelocityAmp, LauncherConstants.ampPower), 
        new FireCommand(true), 
        new WaitCommand(0.5),
        new SetLauncherSpeedCommand(0, 0, 0)
    );

    Command speaker = new SequentialCommandGroup(
        new SetLauncherSpeedCommand(LauncherConstants.upperVelocitySpeaker, LauncherConstants.lowerVelocitySpeaker, 0),
        new FireCommand(true)
    );

    Command toRun;
    /**
     */
    public RunRobotMode() {

    }

    @Override
    public void initialize() {
        
        // Choose which command to run
        switch (Launcher.currentMode) {
            case AMP:
                toRun = amp;
                break;
            case SPEAKER:
                toRun = speaker;
                break;
        }

        toRun.schedule();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
