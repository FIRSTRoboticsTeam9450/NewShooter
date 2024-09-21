package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ResetClimbCommand extends Command {
    
    ClimbSubsystem climb = ClimbSubsystem.getInstance();

    @Override
    public void initialize() {
        climb.runDownAndReset();
    }

    @Override
    public boolean isFinished() {
        return climb.isResetting();
    }

}
