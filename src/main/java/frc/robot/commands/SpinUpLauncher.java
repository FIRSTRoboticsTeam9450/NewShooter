package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.RobotConstants;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.LauncherMode;

/**
 * Based on the mode, it will choose speeds for the launcher then launch the note
 */
public class SpinUpLauncher extends Command { // RENAME PLS
    
    Command amp = new SetLauncherSpeedCommand(RobotConstants.upperVelocityAmp, RobotConstants.lowerVelocityAmp, RobotConstants.ampPower);

    Command speaker = new SetLauncherSpeedCommand(RobotConstants.upperVelocitySpeaker, RobotConstants.lowerVelocitySpeaker, 0);

    Command toRun;
    /**
     */
    public SpinUpLauncher() {

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
