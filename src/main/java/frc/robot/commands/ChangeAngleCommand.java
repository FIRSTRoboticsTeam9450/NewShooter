package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ChangeAngleCommand extends Command {

    Shooter shooter;
    double power;

    public ChangeAngleCommand(Shooter shooter, double power) {
        this.shooter = shooter;
        this.power = power;
    }

    @Override
    public void initialize() {
        shooter.setRotationSpeed(power);
    }

    @Override
    public void execute() {
        //System.out.println("RUNNING!!!");
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setRotationSpeed(0);
        //System.out.println("END.");
    }
    
}
