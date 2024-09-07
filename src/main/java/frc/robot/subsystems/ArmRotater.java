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
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmRotater extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  CANSparkFlex motorRotateLeft = new CANSparkFlex(28, MotorType.kBrushless);
  CANSparkFlex motorRotateRight = new CANSparkFlex(29, MotorType.kBrushless);
  SparkAbsoluteEncoder throughboreRotateLeft = motorRotateLeft.getAbsoluteEncoder();
  SparkAbsoluteEncoder throughboreRotateRight = motorRotateRight.getAbsoluteEncoder();

  private static ArmRotater rotate;
  Timer timer = new Timer();
  //double powerMult = 1;
  double rotateLeftPower;
  double rotateRightPower;
  double targetRotateEncoder = throughboreRotateLeft.getPosition();
  public boolean onAngle = false;
  double currentEncoderValueLeft = 0;
  double currentEncoderValueRight = 0;
  final double kMaxRotateSpeedUp = 1; //.7; // up .5 down -.2
  final double kMaxRotateSpeedDown = -0.8; //-.3;
  static final double kLeftEncoderOffset = 0.51;//.49942
  static final double kRightEncoderOffset = 0.516;//.4945;
  final double kMaxEncoderDifference = .02;
  double previousRotatePower = 0;
  final double kp = 9;
  int periodicCounter = 0;
  int count = 0;

  
  public static ArmRotater getInstance(String name) {
    if(rotate == null) {
      rotate = new ArmRotater();
    }
    System.out.println(name + " aquired arm rotater");
    return rotate;
  }
  public ArmRotater() {

    motorRotateLeft.restoreFactoryDefaults();
    motorRotateRight.restoreFactoryDefaults();

    motorRotateLeft.setInverted(true);
    motorRotateRight.setInverted(false);

    // leftRotateMotor.setIdleMode(IdleMode.kCoast);
    // rightRotateMotor.setIdleMode(IdleMode.kCoast);

    motorRotateLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    motorRotateRight.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);

    //targetRotateEncoder = currentEncoderValueLeft;
    //stop();
    //setShootInfo(currentShooterInfo);
  }

  public void setRotationSpeed(double leftPower, double rightPower) {
    
    motorRotateLeft.set(leftPower);
    motorRotateRight.set(rightPower);

    SmartDashboard.putNumber("rotate speed", leftPower);

  }

  double previousEncoder = currentEncoderValueLeft;
  double boost = 0;
  double boostThreshold = 0.1;
  double count2 = 0;
  double minPower = 0.05;
  public double rotate(double targetEncoderValue, double currentEncoderValue) {
    double error = (targetEncoderValue - currentEncoderValue);
    double power = error * kp + boost;

    if(!onAngle) {
      if(power > 0 && power < minPower) {
        power = minPower;
      }
      else if(power < 0 && power > -minPower) {
        power = -minPower;
      }
    }

    if(power > boostThreshold) {
      boost = .05;
    }
    else if(power < -boostThreshold) {
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
      // can only change 0.004 per loop
      if(power - previousRotatePower > .004) {
        power = previousRotatePower + .004;//.004;
      }
      else if(power - previousRotatePower < -.004) {
        power = previousRotatePower - .004;//.004;
      }
    }
    previousRotatePower = power;
    previousEncoder = currentEncoderValue;
    return power;
  }



  // zero basing encoders left encoder has .497 offset, right has ?

  public double getLeftRotateEncoder() {
    return kLeftEncoderOffset - throughboreRotateLeft.getPosition() /*- kLeftEncoderOffset*/;
  }

  public double getRightRotateEncoder() {
    return kRightEncoderOffset - throughboreRotateRight.getPosition() /*- kRightEncoderOffset*/;
  }

  public int setRotationTarget(double encoderValue) {
    if(encoderValue != Double.MAX_VALUE) {
      targetRotateEncoder = encoderValue;
    }
    if(encoderValue == Double.MIN_VALUE) {
      System.out.println("Set target correctly");
      targetRotateEncoder = getLeftRotateEncoder();
    }
    manageFlags();

    return periodicCounter;
  }


  private void manageRotate() {
    currentEncoderValueLeft = getLeftRotateEncoder();
    currentEncoderValueRight = getRightRotateEncoder();
    if (targetRotateEncoder > (0.708 - 0.497)) {
      targetRotateEncoder = 0.708 - 0.497;
    }
    if (targetRotateEncoder < 0) {
      targetRotateEncoder = 0;
    }

    rotateLeftPower = rotate(targetRotateEncoder, currentEncoderValueLeft);
    rotateRightPower = rotate(targetRotateEncoder, currentEncoderValueRight);
    if(Math.abs(currentEncoderValueLeft - currentEncoderValueRight) > kMaxEncoderDifference) {
      rotateLeftPower = 0;
      rotateRightPower = 0;
      System.out.println("rotate encoder misaligned");
    }
    setRotationSpeed(rotateLeftPower, rotateRightPower);
  }

  private void manageFlags() {

    if(Math.abs(targetRotateEncoder - currentEncoderValueLeft) < .001) {
      onAngle = true;
    }
    else if(currentEncoderValueLeft > targetRotateEncoder && previousEncoder < targetRotateEncoder) {
      onAngle = true;
    }
    else if(currentEncoderValueLeft < targetRotateEncoder && previousEncoder > targetRotateEncoder) {
      onAngle = true;
    }
    else {
      onAngle = false;
    }
    
  }

  public int getPeriodicCounter() {
    return periodicCounter;
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
    SmartDashboard.putNumber("Target left Encoder", targetRotateEncoder);
    SmartDashboard.putBoolean("On angle", onAngle);
    //System.out.println(currentShooterInfo.intakeSpeed + " INTAKE SPEED");
}

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}