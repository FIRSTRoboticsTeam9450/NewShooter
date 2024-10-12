package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;

public class ProcNoteCommand extends Command {

    Intake intake = Intake.getInstance("ProcNoteCommand");
    Launcher launcher = Launcher.getInstance("ProcNoteCommand");

    public static boolean scheduled = false;
    public static boolean runningIntake = false;

    Timer timeout = new Timer();
    boolean running;

    @Override
    public void initialize() {
        scheduled = true;
        System.out.println("PROCESS NOTE COMMAND SCHEDULED");
        timeout.restart();
        addRequirements(launcher);
        addRequirements(intake);
        running = false;
    }

    @Override
    public boolean isFinished() {
        return running && intake.getExitLaserDistance() >= 150;
    }

    @Override
    public void execute() {
        if (intake.getExitLaserDistance() < 150 && !running) {
            System.out.println("RUNNING INTAKE");
            runningIntake = true;
            intake.setPower(-0.1);
            running = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        scheduled = false;
        runningIntake = false;
        intake.setPower(0);
        launcher.setVelocities(0, 0);
    }


    
}
