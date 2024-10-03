package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Controls the two primary launcher motors */
public class Launcher extends SubsystemBase {

    CANSparkFlex motorUpper = new CANSparkFlex(30, MotorType.kBrushless);
    CANSparkFlex motorLower = new CANSparkFlex(31, MotorType.kBrushless);
    CANSparkFlex ampMotor = new CANSparkFlex(35, MotorType.kBrushless);
    RelativeEncoder encoderUpper = motorUpper.getEncoder();
    RelativeEncoder encoderLower = motorLower.getEncoder();

    public static LauncherMode currentMode = LauncherMode.SPEAKER;

    // PIDF Config
    PIDController upperController = new PIDController(0.0018, 0.004, 0);
    PIDController lowerController = new PIDController(0.0018, 0.004, 0);
    static final double upperKv = 0.00174;
    static final double lowerKv = 0.0018;

    double upperPower = 0;
    double lowerPower = 0;

    boolean atSpeed = true;
    double atSpeedCounter = 0;

    boolean usePID = true;

    Timer noPidTimer = new Timer();

    private static Launcher launcher;

    /** Creates a new Launcher object */
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

    /**
     * Returns the instance of Launcher, ensuring only one copy is made
     * @param name The name of the class requesting the Launcher instance
     * @return An instance of Launcher
     */
    public static Launcher getInstance(String name) {
        if (launcher == null) {
            launcher = new Launcher();
        }
        System.out.println(name + " has acquired Launcher");
        return launcher;
    }

    /** Updates the Launcher PIDF controllers */
    private void updatePIDs() {
        double upperError = Math.abs(upperController.getSetpoint() - encoderUpper.getVelocity());
        double lowerError = Math.abs(lowerController.getSetpoint() - encoderLower.getVelocity());

        // Count how many loops the RPM of the motors is in a good range - used to know when velocity is stable
        if (upperError < 20 && lowerError < 20) {
            atSpeedCounter++;
        } else {
            atSpeedCounter = 0;
        }

        upperPower = upperController.getSetpoint() * upperKv;
        lowerPower = lowerController.getSetpoint() * lowerKv;

        // Only run PID controller when within 300 RPM of target - feedforward is enough until that point
        if (upperError < 300) {
            upperPower += upperController.calculate(encoderUpper.getVelocity());
        }
        if (lowerError < 300) {
            lowerPower += lowerController.calculate(encoderLower.getVelocity());
        }

        upperPower = MathUtil.clamp(upperPower, -12, 12);
        lowerPower = MathUtil.clamp(lowerPower, -12, 12);

        motorUpper.setVoltage(upperPower);
        motorLower.setVoltage(lowerPower);
    }

    /**
     * Sets the target velocities in RPM of the Launcher motors
     * @param upper The desired RPM of the top motor
     * @param lower The desired RPM of the bottom motor
     */
    public void setVelocities(double upper, double lower) {
        upperController.setSetpoint(upper);
        lowerController.setSetpoint(lower);
    }

    public void setAmpPower(double power) {
        ampMotor.set(power);
    }

    /**
     * Sets the powers of the Launcher motors. Only works if PID is disabled.
     * @param upper The desired power of the top motor (-1 to 1)
     * @param lower The desired power of the bottom motor (-1 to 1)
     */
    public void setPowers(double upper, double lower) {
        motorUpper.set(upper);
        motorLower.set(lower);
        noPidTimer.restart();
    }

    /** Returns whether the Launcher is running at a stable and accurate speed */
    public boolean atSpeed() {
        return atSpeedCounter > 20 && upperController.getSetpoint() != 0 && lowerController.getSetpoint() != 0;
    }

    /**
     * Returns whether the Launcher is running at a stable and accurate speed
     * @param tolerance The number of loops the Launcher needs to be at a stable RPM for. If 0, returns true as soon as wheels spin up.
     * @return The Launcher's state
     */
    public boolean atSpeed(int tolerance) {
        if (!usePID) {
            return noPidTimer.hasElapsed(1);
        }
        if (tolerance == 0) {
            double upperError = Math.abs(upperController.getSetpoint() - encoderUpper.getVelocity());
            return (upperError < 300) && upperController.getSetpoint() != 0 && lowerController.getSetpoint() != 0; 
        }
        return atSpeedCounter > tolerance && upperController.getSetpoint() != 0 && lowerController.getSetpoint() != 0;
    }

    @Override
    public void periodic() {
        if (usePID) {
            updatePIDs();
        }

        // Logging to AdvantageScope
        Logger.recordOutput("Launcher/UpperVelocity", encoderUpper.getVelocity());
        Logger.recordOutput("Launcher/UpperSetpoint", upperController.getSetpoint());
        Logger.recordOutput("Launcher/UpperVoltage", upperPower);

        Logger.recordOutput("Launcher/LowerSetpoint", lowerController.getSetpoint());
        Logger.recordOutput("Launcher/LowerVelocity", encoderLower.getVelocity());
        Logger.recordOutput("Launcher/LowerVoltage", lowerPower);

        Logger.recordOutput("Launcher/atSpeed", atSpeed());

    }
    
}