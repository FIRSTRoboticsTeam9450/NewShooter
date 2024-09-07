package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;

public class FireCommand extends Command {

    Intake intake = Intake.getInstance("FireCommand");
    Launcher launcher = Launcher.getInstance("FireCommand");

    boolean fired;
    boolean dontRun;
    boolean dontStabilize;

    Timer timer = new Timer();

    public FireCommand() {

    }

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
        return (fired && timer.hasElapsed(1)) || dontRun;
    }

    @Override
    public void end(boolean interrupted) {
        launcher.setVelocities(0, 0);
        intake.setPower(0);
    }

}