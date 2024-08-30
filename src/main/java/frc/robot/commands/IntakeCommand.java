package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command {

    Intake intake = Intake.getInstance("IntakeCommand");
    double power;

    public IntakeCommand(double power) {
        this.power = power;
    }

    @Override
    public void initialize() {
        intake.setPower(power);
    }

    @Override
    public boolean isFinished() {
        return intake.getEntryLaserDistance() <= 60;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setPower(0);
    }
    
}
