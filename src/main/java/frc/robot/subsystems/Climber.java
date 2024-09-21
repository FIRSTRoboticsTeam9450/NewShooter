package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    CANSparkFlex motor;
    SparkLimitSwitch upperLimit;
    SparkLimitSwitch lowerLimit;
    RelativeEncoder encoder;

    double power;

    boolean runPid;
    boolean resetting;

    PIDController pid = new PIDController(0.6, 0, 0);

    public Climber(int id) {
        motor = new CANSparkFlex(id, MotorType.kBrushless);
        encoder = motor.getEncoder();
        encoder.setPosition(0);

        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(true);

        upperLimit = motor.getForwardLimitSwitch(Type.kNormallyOpen);
        lowerLimit = motor.getReverseLimitSwitch(Type.kNormallyOpen);

        pid.setSetpoint(encoder.getPosition());
    }

    public void setVoltage(double voltage) {
        power = voltage;
        runPid = false;
    }

    public void setSetpoint(double setpoint) {
        pid.setSetpoint(setpoint);
        runPid = true;
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    private void updatePid() {
        power = pid.calculate(encoder.getPosition());
        power = MathUtil.clamp(power, -8, 8);
    }

    public boolean atPosition() {
        return Math.abs(pid.getSetpoint() - encoder.getPosition()) < 1;
    }

    public boolean lowerLimitSwitch() {
        return upperLimit.isPressed();
    }

    public void reset() {
        setVoltage(1);
        resetting = true;
    }

    public void resetEncoder() {
        encoder.setPosition(0);
    }

    public boolean isResetting() {
        return resetting;
    }

    @Override
    public void periodic() {
        if (runPid) {
            updatePid();
        }

        if (lowerLimitSwitch()) {
            encoder.setPosition(1);
            resetting = false;
        }

        motor.setVoltage(power);
    }
    
}
