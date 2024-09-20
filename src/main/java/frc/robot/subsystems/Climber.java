package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    CANSparkFlex motor;
    RelativeEncoder encoder;

    boolean runPid;

    PIDController pid = new PIDController(0.6, 0, 0);

    public Climber(int id) {
        motor = new CANSparkFlex(id, MotorType.kBrushless);
        encoder = motor.getEncoder();
        encoder.setPosition(0);

        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(false);

        pid.setSetpoint(encoder.getPosition());
    }

    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
        runPid = false;
    }

    public void setSetpoint(double setpoint) {
        pid.setSetpoint(setpoint);
        runPid = true;
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    public void reset() {
        encoder.setPosition(0);
    }

    private void updatePid() {
        double power = pid.calculate(encoder.getPosition());
        power = MathUtil.clamp(power, -8, 8);
        motor.setVoltage(power);
    }

    public boolean atPosition() {
        return Math.abs(pid.getSetpoint() - encoder.getPosition()) < 1;
    }

    @Override
    public void periodic() {
        if (runPid) {
            updatePid();
        }
    }
    
}
