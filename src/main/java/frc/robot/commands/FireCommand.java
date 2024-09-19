package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;

/** Fires a note when the intake has a note and the launcher is at speed */
public class FireCommand extends Command {

    Intake intake = Intake.getInstance("FireCommand");
    Launcher launcher = Launcher.getInstance("FireCommand");

    boolean fired;
    boolean dontRun;
    boolean dontStabilize;

    Timer timer = new Timer();

    
    public FireCommand() {

    }

    /**
     * Creates a new FireCommand
     * @param dontStabilize Causes the robot to fire the note as soon as the current RPM is close to the target RPM (faster, less accurate)
     */
    public FireCommand(boolean dontStabilize) {
        this.dontStabilize = dontStabilize;
    }

    @Override
    public void initialize() {
        addRequirements(intake);
        addRequirements(launcher);
        intake.setPower(0);
        fired = false;
        dontRun = false;
        if (intake.getEntryLaserDistance() > 150) {
            dontRun = true;
        }
    }

    @Override
    public void execute() {
        int tolerance = dontStabilize ? 0 : 20;
        if (launcher.atSpeed(tolerance) && !fired) {
            intake.setPower(1);
            timer.restart();
            fired = true;
        }
    }

    @Override
    public boolean isFinished() {
        // stop the wheels 1 second after the note was fired
        return (fired && timer.hasElapsed(0.5)) || dontRun;
    }

    @Override
    public void end(boolean interrupted) {
        launcher.setVelocities(0, 0);
        intake.setPower(0);
    }

}