package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class ReverseIntakeCommand extends Command {

        Intake intake = Intake.getInstance("IntakeNoteCommand");
        double power;

        public ReverseIntakeCommand(double power) {
            this.power = -power;
        }

        @Override
        public void initialize() {
            intake.setPower(power);
        }

        @Override
        public void end(boolean interrupted) {
            intake.setPower(0);
        }

    
}
