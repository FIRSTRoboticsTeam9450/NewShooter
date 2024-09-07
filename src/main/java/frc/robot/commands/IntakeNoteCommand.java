package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/** Runs the intake until a note is detected inside */
public class IntakeNoteCommand extends Command {

    Intake intake = Intake.getInstance("IntakeNoteCommand");

    // If the sensor reads less than this value (in mm), a note is detected
    double triggerDistance = 150;

    @Override
    public void initialize() {
        addRequirements(intake);
        if (intake.getEntryLaserDistance() > triggerDistance) {
            intake.setPower(1);
        }
    }

    @Override
    public void execute() {
        if (intake.getEntryLaserDistance() > triggerDistance) {
            intake.setPower(1);
        }
    }

    @Override
    public boolean isFinished() {
        return intake.getEntryLaserDistance() < triggerDistance;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setPower(0);
    }
    
}
