package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Carriage;

public class SetLauncherAngle extends Command {

    double encoderValue;
    Carriage carriage = Carriage.getInstance("SetLauncherAngle");

    public SetLauncherAngle(double encoderValue) {
        this.encoderValue = encoderValue;
    }

    @Override
    public void initialize() {
        carriage.setTarget(encoderValue);
    }

    @Override
    public boolean isFinished() {
        return carriage.onAngle;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
    
}
