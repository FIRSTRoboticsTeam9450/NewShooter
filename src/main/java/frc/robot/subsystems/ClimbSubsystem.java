package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {

    Climber lClimb = new Climber(40); // radio
    Climber rClimb = new Climber(41); // radio

    private static ClimbSubsystem climbSub;

    public static ClimbSubsystem getInstance() {
        if (climbSub == null) {
            climbSub = new ClimbSubsystem();
        }
        return climbSub;
    }

    public void setSetpoint(double setpoint) {
        lClimb.setSetpoint(setpoint);
        rClimb.setSetpoint(setpoint);
    }

    public void setVoltages(double left, double right) {
        lClimb.setVoltage(left);
        rClimb.setVoltage(right);
    }

    public void setLeftVoltage(double voltage) {
        lClimb.setVoltage(voltage);
    }

    public void setRightVoltage(double voltage) {
        rClimb.setVoltage(voltage);
    }

    public void reset() {
        lClimb.reset();
        rClimb.reset();
    }

    public boolean atPosition() {
        return lClimb.atPosition() && rClimb.atPosition();
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Climb/Left Position", lClimb.getPosition());
        Logger.recordOutput("Climb/Right Position", rClimb.getPosition());

    }
    
}
