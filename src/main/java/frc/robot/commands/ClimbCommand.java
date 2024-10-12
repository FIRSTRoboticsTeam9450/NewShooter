package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.RobotConstants.ClimbConstants;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCommand extends Command {

    ClimbSubsystem climb = ClimbSubsystem.getInstance();
    double setpoint;

    boolean amp;

    public ClimbCommand(double setpoint) {
        addRequirements(climb);
        this.setpoint = setpoint;
    }

    public ClimbCommand(double setpoint, boolean amp) {
        this(setpoint);
        this.amp = amp;
    }

    @Override
    public void initialize() {
        climb.setSetpoint(setpoint);
        if (amp) {
            climb.setMaxVoltage(ClimbConstants.ampVoltage);
        } else {
            climb.setMaxVoltage(ClimbConstants.climbVoltage);
        }
    }

    @Override
    public boolean isFinished() {
        return climb.atPosition();
    }
    
}
