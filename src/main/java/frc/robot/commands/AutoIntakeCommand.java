package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class AutoIntakeCommand extends Command {

    Intake intake = Intake.getInstance("AutoIntakeCommand");

    @Override
    public void initialize() {
        addRequirements(intake);
        if (intake.getEntryLaserDistance() > 150) {
            intake.setPower(1);
        }
    }

    @Override
    public void execute() {
        if (intake.getEntryLaserDistance() > 150) {
            intake.setPower(1);
        }
    }

    @Override
    public boolean isFinished() {
        return intake.getEntryLaserDistance() < 150;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setPower(0);
    }
    
}
