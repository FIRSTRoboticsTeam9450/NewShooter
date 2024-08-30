package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeSetPowerCommand extends Command {
    Intake intake = Intake.getInstance("IntakeSetPowerCommand");
    double power;

    public IntakeSetPowerCommand(double power) {
        this.power = power;
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
