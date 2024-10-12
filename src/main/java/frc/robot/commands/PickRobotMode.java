package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.RobotConstants.ClimbConstants;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.LauncherMode;

/**
 * gfdsfdsafdsa - Sets the mode and based on that mode moves the climbers
 */
public class PickRobotMode extends Command { // RENAME PLS

    Launcher launcher = Launcher.getInstance("SetRobotMode");
    LauncherMode mode = LauncherMode.SPEAKER;

    // Climb commands based off of amp or not
    ClimbCommand climbAmp = new ClimbCommand(ClimbConstants.ampPos, true);
    ClimbCommand climbDown = new ClimbCommand(ClimbConstants.downPos, true);

    /** Change the current mode to the mode wanted */
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
