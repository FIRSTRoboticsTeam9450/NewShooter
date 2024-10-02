package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.LauncherMode;

public class SpinUpAmp extends Command { // RENAME PLS

    Launcher launcher = Launcher.getInstance("SetRobotMode");
    LauncherMode current = new LauncherMode();
    final LauncherMode amp = new LauncherMode(.5, .5, 1, true);
    final LauncherMode subwoofer = new LauncherMode(1, 1, 0, true);

    /**
     */
    public SpinUpAmp(boolean spin) {

    }

    @Override
    public void initialize() {
        Command x = new SequentialCommandGroup(new ClimbCommand(current.getClimb()), new SetLauncherSpeedCommand(current.getUpper(), current.getLower()), );
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
