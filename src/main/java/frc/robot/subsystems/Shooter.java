// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  CANSparkFlex intakeMotor = new CANSparkFlex(32, MotorType.kBrushless);
  CANSparkFlex upperShooterMotor = new CANSparkFlex(30, MotorType.kBrushless);
  CANSparkFlex lowerShooterMotor = new CANSparkFlex(31, MotorType.kBrushless);
  CANSparkMax leftRotateMotor = new CANSparkMax(28, MotorType.kBrushless);
  CANSparkMax rightRotateMotor = new CANSparkMax(29, MotorType.kBrushless);
  RelativeEncoder upperEncoder = upperShooterMotor.getEncoder();
  RelativeEncoder lowerEncoder = lowerShooterMotor.getEncoder();
  SparkAbsoluteEncoder leftRotateThroughbore = leftRotateMotor.getAbsoluteEncoder();
  SparkAbsoluteEncoder rightRotateThroughbore = rightRotateMotor.getAbsoluteEncoder();

  private static Shooter shooter;
  Timer timer = new Timer();
  boolean started = false;
  boolean intakeOn = false;
  boolean noteIn = false;
  double normalUpper = 0;
  double normalLower = 0;
  double upperSpeed = 5676;
  double lowerSpeed = 6784;
    

  public static Shooter getInstance(String name) {
    if(shooter == null) {
      shooter = new Shooter();
    }
    System.out.println(name + " aquired Shooter");
    return shooter;
  }
  public Shooter() {
    upperShooterMotor.restoreFactoryDefaults();
    lowerShooterMotor.restoreFactoryDefaults();

    leftRotateMotor.restoreFactoryDefaults();
    rightRotateMotor.restoreFactoryDefaults();

    intakeMotor.restoreFactoryDefaults();

    upperShooterMotor.setInverted(false);
    lowerShooterMotor.setInverted(true);

    leftRotateMotor.setInverted(false);
    rightRotateMotor.setInverted(true);

    upperShooterMotor.setIdleMode(IdleMode.kCoast);
    lowerShooterMotor.setIdleMode(IdleMode.kCoast);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command shoot() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          System.out.println("Shoot");
          /* one-time action goes here */
        });
  }
  
  public void initializeShoot() {
    started = false;
    intakeOn = false;
    noteIn = false;
    normalUpper = 0;
    normalLower = 0;
    timer.reset();

  }
  public boolean runShooterMotors() {
    SmartDashboard.putNumber("Upper Speed", upperEncoder.getVelocity());
    SmartDashboard.putNumber("Lower Speed", lowerEncoder.getVelocity());

    upperShooterMotor.set(1);
    lowerShooterMotor.set(1);
    if(!started) {
      timer.reset();
      timer.start();
      started = true;
    }
    if(intakeOn) {
      if(upperEncoder.getVelocity() - normalUpper < -600 || lowerEncoder.getVelocity() - normalLower < -600) {
        noteIn = true;
      }

      if(noteIn) {
        if(upperEncoder.getVelocity() >= normalUpper || lowerEncoder.getVelocity() >= normalUpper) {
          return true;
        }
      }
    }
    if(timer.hasElapsed(1)) {
      timer.stop();
      normalUpper = upperEncoder.getVelocity();
      normalLower = lowerEncoder.getVelocity();
      intakeMotor.set(-1);
      intakeOn = true;
    }
    return false;
  }

  public void setRotationSpeed(double power, double encoderValue) {
    if(encoderValue > leftRotateThroughbore.getPosition()) {
      power *= -1;
    }
    leftRotateMotor.set(power);
    rightRotateMotor.set(power);

  }
  public boolean rotateTil(double encoderValue) {
    if(Math.abs(encoderValue - leftRotateThroughbore.getPosition()) > .05) {
      return false;
    } 
    return true;
  }
  public void setIntakeSpeed(double power) {
    intakeMotor.set(power);
  }

  public void resetMotors() {
    intakeMotor.set(0);
    upperShooterMotor.set(0);
    lowerShooterMotor.set(0);
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {

    //System.out.println("Worked");
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("left throughbore position", leftRotateThroughbore.getPosition());
    SmartDashboard.putNumber("left throughbore velocity", leftRotateThroughbore.getVelocity());
    SmartDashboard.putNumber("right throughbore position", rightRotateThroughbore.getPosition());
    SmartDashboard.putNumber("right throughbore velocity", rightRotateThroughbore.getVelocity());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
