package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {

    CANSparkFlex motorUpper = new CANSparkFlex(30, MotorType.kBrushless);
    CANSparkFlex motorLower = new CANSparkFlex(31, MotorType.kBrushless);
    RelativeEncoder encoderUpper = motorUpper.getEncoder();
    RelativeEncoder encoderLower = motorLower.getEncoder();

    PIDController upperController = new PIDController(0.0015, 0.005, 0);
    PIDController lowerController = new PIDController(0.00, 0, 0);

    static final double upperKv = 0.00172;
    static final double lowerKv = 0;

    double upperPower = 0;
    double lowerPower = 0;

    boolean atSpeed = true;

    boolean usePID = true;

    private static Launcher launcher;

    public Launcher() {
        motorUpper.restoreFactoryDefaults();
        motorLower.restoreFactoryDefaults();

        motorUpper.setIdleMode(IdleMode.kCoast);
        motorLower.setIdleMode(IdleMode.kCoast);

        motorUpper.setInverted(false);
        motorLower.setInverted(true);

        upperController.setSetpoint(0);
        lowerController.setSetpoint(0);
    }

    public static Launcher getInstance(String name) {
        if (launcher == null) {
            launcher = new Launcher();
        }
        System.out.println(name + " has acquired Launcher");
        return launcher;
    }

    public void updatePIDs() {
        upperPower = upperController.getSetpoint() * upperKv;
        lowerPower = lowerController.getSetpoint() * lowerKv;

        if (Math.abs(upperController.getSetpoint() - encoderUpper.getVelocity()) < 300) {
            upperPower += upperController.calculate(encoderUpper.getVelocity());
        }
        if (Math.abs(lowerController.getPositionError()) < 300) {
            lowerPower += lowerController.calculate(encoderLower.getVelocity());
        }

        upperPower = MathUtil.clamp(upperPower, -12, 12);
        lowerPower = MathUtil.clamp(lowerPower, -12, 12);

        motorUpper.setVoltage(upperPower);
        motorLower.setVoltage(lowerPower);
    }

    public void setVelocities(double upper, double lower) {
        upperController.setSetpoint(upper);
        lowerController.setSetpoint(lower);
    }

    public void setPowers(double upper, double lower) {
        motorUpper.set(upper);
        motorLower.set(lower);
    }

    public boolean atSpeed() {
        return atSpeed;
    }

    @Override
    public void periodic() {
        if (usePID) {
            updatePIDs();
        }

        Logger.recordOutput("Launcher/UpperVelocity", encoderUpper.getVelocity());
        Logger.recordOutput("Launcher/UpperSetpoint", upperController.getSetpoint());
        Logger.recordOutput("Launcher/UpperVoltage", upperPower);

        Logger.recordOutput("Launcher/LowerSetpoint", lowerController.getSetpoint());
        Logger.recordOutput("Launcher/LowerVelocity", encoderLower.getVelocity());
        Logger.recordOutput("Launcher/LowerVoltage", lowerPower);

    }
    
}