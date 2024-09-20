package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCommand extends Command {

    ClimbSubsystem climb = ClimbSubsystem.getInstance();
    double setpoint;

    public ClimbCommand(double setpoint) {
        addRequirements(climb);
        this.setpoint = setpoint;
    }

    @Override
    public void initialize() {
        climb.setSetpoint(setpoint);
    }

    @Override
    public boolean isFinished() {
        return climb.atPosition();
    }
    
}
