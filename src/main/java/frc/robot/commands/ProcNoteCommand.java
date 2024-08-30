package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class ProcNoteCommand extends Command {

    Intake intake = Intake.getInstance("ProcNoteCommand");
    boolean backUp = false;
    boolean finished = false;

    public ProcNoteCommand() {

    }

    @Override
    public void initialize() {
        if (intake.getExitLaserDistance() <= 60) {
            backUp = true;
        }
        if (intake.getEntryLaserDistance() >= 60) {
            finished = true;
            System.out.println("NO NOTE!!");
        }
    }

    @Override
    public void execute() {
        if (backUp) {
            intake.setPower(-0.05);
            if (intake.getExitLaserDistance() >= 60) {
                backUp = false;
            }
        } else {
            intake.setPower(0.05);
            if (intake.getExitLaserDistance() <= 60) {
                finished = true;
                intake.setPower(0);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setPower(0);
    }
    
}
