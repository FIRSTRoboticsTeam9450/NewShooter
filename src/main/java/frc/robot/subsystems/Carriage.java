// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Carriage extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  CANSparkMax motorRotateLeft = new CANSparkMax(28, MotorType.kBrushless);
  CANSparkMax motorRotateRight = new CANSparkMax(29, MotorType.kBrushless);
  SparkAbsoluteEncoder throughboreRotateLeft = motorRotateLeft.getAbsoluteEncoder();
  SparkAbsoluteEncoder throughboreRotateRight = motorRotateRight.getAbsoluteEncoder();

  private static Carriage carriage;
  double rotateLeftPower;
  double rotateRightPower;
  public boolean onAngle = false;
  double currentEncoderValueLeft = 0;
  double currentEncoderValueRight = 0;
  final double kMaxRotateSpeedUp = .7; // up .5 down -.2
  final double kMaxRotateSpeedDown = -.3;
  static final double kLeftEncoderOffset = .49942;
  final double kRightEncoderOffset = .4945;
  final double kMaxEncoderDifference = .02;
  double previousRotatePower = 0;
  final double kp = 6;
  int periodicCounter = 0;

  double target;

  public static Carriage getInstance(String name) {
    if(carriage == null) {
      carriage = new Carriage();
    }
    System.out.println(name + " acquired Carriage");
    return carriage;
  }
  public Carriage() {
    motorRotateLeft.restoreFactoryDefaults();
    motorRotateRight.restoreFactoryDefaults();

    motorRotateLeft.setInverted(true);
    motorRotateRight.setInverted(false);
    // leftRotateMotor.setIdleMode(IdleMode.kCoast);
    // rightRotateMotor.setIdleMode(IdleMode.kCoast);

    motorRotateLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    motorRotateRight.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);

    //stop();
  }
  
  public void initializeShoot() {
    onAngle = false;
    previousRotatePower = 0;
  }

  public void setRotationSpeed(double leftPower, double rightPower) {
    
    motorRotateLeft.set(leftPower);
    motorRotateRight.set(rightPower);

    SmartDashboard.putNumber("rotate speed", leftPower);

  }

  double previousEncoder = currentEncoderValueLeft;
  double boost = 0;
  double count2 = 0;
  public double rotate(double targetEncoderValue, double currentEncoderValue) {
    double error = (targetEncoderValue - currentEncoderValue);
    double power = error * kp + boost;

    if(!onAngle) {
      if(power > 0 && power < .05) {
        power = .05;
      }
      else if(power < 0 && power > -.05) {
        power = -.05;
      }
    }

    if(power > .1) {
      boost = .05;
    }
    else if(power < -.1) {
      boost = -.05;
    }

    if(currentEncoderValue > targetEncoderValue && previousEncoder < targetEncoderValue) {
      boost = 0;
    }
    else if(currentEncoderValue < targetEncoderValue && previousEncoder > targetEncoderValue) {
      boost = 0;
    }
    if(error >= 0) {
      power = Math.min(kMaxRotateSpeedUp, power); // Limiting power when going up
    }
    else{
      power = Math.max(kMaxRotateSpeedDown, power); // Limiting power when going down
    }
    if(Math.abs(power) > .1) {
      if(Math.signum(previousRotatePower) != Math.signum(power)) {
        previousRotatePower = 0;
      }
      if(power - previousRotatePower > .01) {
        power = previousRotatePower + .004;
      }
      else if(power - previousRotatePower < -.01) {
        power = previousRotatePower - .004;
      }
    }
    previousRotatePower = power;
    previousEncoder = currentEncoderValue;
    return power;
  }

  // zero basing encoders left encoder has .497 offset, right has ?

  public double getLeftRotateEncoder() {
    return throughboreRotateLeft.getPosition() - kLeftEncoderOffset;
  }

  public double getRightRotateEncoder() {
    return throughboreRotateRight.getPosition() - kRightEncoderOffset;
  }

  public void stop() {

  }

  private void manageRotate() {
    currentEncoderValueLeft = getLeftRotateEncoder();
    currentEncoderValueRight = getRightRotateEncoder();
    if (target > (0.708 - 0.497)) {
      target = 0.708 - 0.497;
    }
    if (target < 0) {
      target = 0;
    }

    rotateLeftPower = rotate(target, currentEncoderValueLeft);
    rotateRightPower = rotate(target, currentEncoderValueRight);
    if(Math.abs(currentEncoderValueLeft - currentEncoderValueRight) > kMaxEncoderDifference) {
      rotateLeftPower = 0;
      rotateRightPower = 0;
      System.out.println("rotate encoder misaligned");
    }
    setRotationSpeed(rotateLeftPower, rotateRightPower);
  }

  private void manageFlags() {

    if(Math.abs(target - currentEncoderValueLeft) < .001) {
      onAngle = true;
    }
    else if(currentEncoderValueLeft > target && previousEncoder < target) {
      onAngle = true;
    }
    else if(currentEncoderValueLeft < target && previousEncoder > target) {
      onAngle = true;
    }
    else {
      onAngle = false;
    }
  }

  public int getPeriodicCounter() {
    return periodicCounter;
  }

  public void setTarget(double target) {
    this.target = target;
  }

  @Override
  public void periodic() {

    manageRotate();

    manageFlags();
    periodicCounter++;
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("left throughbore position", currentEncoderValueLeft);
    //SmartDashboard.putNumber("Current Left Throughbore", leftRotateThroughbore.getPosition());
    SmartDashboard.putNumber("right throughbore position", currentEncoderValueRight);
    SmartDashboard.putBoolean("On angle", onAngle);
    //System.out.println(currentShooterInfo.intakeSpeed + " INTAKE SPEED");
}

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
