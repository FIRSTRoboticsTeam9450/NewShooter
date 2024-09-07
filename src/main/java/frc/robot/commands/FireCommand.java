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

    Timer timer = new Timer();

    @Override
    public void initialize() {
        intake.setPower(0);
        fired = false;
        if (intake.getEntryLaserDistance() > 150) {
            dontRun = true;
        }
    }

    @Override
    public void execute() {
        if (launcher.atSpeed() && !fired) {
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
        if (!dontRun) {
            launcher.setVelocities(0, 0);
            intake.setPower(0);
        }
    }

}