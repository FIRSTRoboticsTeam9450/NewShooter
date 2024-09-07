package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class ProcNoteCommand extends Command {

    Intake intake = Intake.getInstance("ProcNoteCommand");;

    @Override
    public void initialize() {
        if (intake.getExitLaserDistance() < 150) {
            intake.setPower(-0.15);
        }
    }

    @Override
    public boolean isFinished() {
        return intake.getExitLaserDistance() > 150;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setPower(0);
    }


    
}
