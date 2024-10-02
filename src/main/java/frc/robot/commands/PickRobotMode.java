package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.LauncherMode;

/**
 * gfdsfdsafdsa - Sets the mode and based on that mode moves the climbers
 */
public class PickRobotMode extends Command { // RENAME PLS

    Launcher launcher = Launcher.getInstance("SetRobotMode");
    LauncherMode mode = LauncherMode.SPEAKER;

    ClimbCommand climbAmp = new ClimbCommand(60);
    ClimbCommand climbDown = new ClimbCommand(0);

    public PickRobotMode(LauncherMode mode) {
        this.mode = mode;
    }

    @Override
    public void initialize() {

        Launcher.currentMode = mode;
        switch (Launcher.currentMode) {
            case AMP:
                climbAmp.schedule();
                break;
            case SPEAKER:
                climbDown.schedule();
                break;

        }
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
