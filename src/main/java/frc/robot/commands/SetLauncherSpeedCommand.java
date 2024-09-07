package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;

/** Sets the speeds of the launcher wheels in RPM */
public class SetLauncherSpeedCommand extends Command {

    Launcher launcher = Launcher.getInstance("SetLauncherSpeedCommand");

    double upperVelocity;
    double lowerVelocity;

    /**
     * Creates a new SetLauncherSpeedCommand
     * @param upperVelocity The desired RPM of the top motor
     * @param lowerVelocity The desired RPM of the bottom motor
     */
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
