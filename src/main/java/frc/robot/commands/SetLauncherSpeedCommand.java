package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;

public class SetLauncherSpeedCommand extends Command {

    Launcher launcher = Launcher.getInstance("SetLauncherSpeedCommand");

    double upperVelocity;
    double lowerVelocity;

    public SetLauncherSpeedCommand(double upperVelocity, double lowerVelocity) {
        this.upperVelocity = upperVelocity;
        this.lowerVelocity = lowerVelocity;
    }

    @Override
    public void initialize() {
        launcher.setVelocities(upperVelocity, lowerVelocity);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
