package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SetAngleCommand extends Command {

    Shooter shooter;
    double power;
    double distance;
    double encoderValue;
    public SetAngleCommand(Shooter shooter, double power, double distance, double encoderValue) {
        this.shooter = shooter;
        this.power = power;
        this.distance = distance;
        this.encoderValue = encoderValue;
        
    }

    @Override
    public void initialize() {
        shooter.setRotationSpeed(power, encoderValue);
    }

    @Override
    public void execute() {
        //System.out.println("RUNNING!!!");
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setRotationSpeed(0, 0);
        //System.out.println("END.");
    }
    
    @Override
    public boolean isFinished() {
        return shooter.rotateTil(encoderValue);
    }
}
