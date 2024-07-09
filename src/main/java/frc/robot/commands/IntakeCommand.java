package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class IntakeCommand extends Command {
    Shooter shooter;
    double power;

    public IntakeCommand(Shooter shooter, double power) {
        this.shooter = shooter;
        this.power = power;
    }

    @Override
    public void initialize() {
        shooter.setIntakeSpeed(power);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setIntakeSpeed(0);
    }
}
