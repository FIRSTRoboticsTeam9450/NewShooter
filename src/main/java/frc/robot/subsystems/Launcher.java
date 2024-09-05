package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
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

    PIDController upperController = new PIDController(0.01, 0, 0);
    PIDController lowerController = new PIDController(0.01, 0, 0);

    boolean atSpeed = true;

    boolean usePID = false;

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
        double upperPower = upperController.calculate(encoderUpper.getVelocity());
        double lowerPower = lowerController.calculate(encoderLower.getVelocity());

        upperPower = MathUtil.clamp(upperPower, -1, 1);
        lowerPower = MathUtil.clamp(lowerPower, -1, 1);

        motorUpper.set(upperPower);
        motorLower.set(lowerPower);
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
    }
    
}