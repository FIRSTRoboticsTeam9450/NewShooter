// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.RobotConstants;
import frc.RobotConstants.ArmConstants;

public class ArmRotater extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  CANSparkFlex motorRotateLeft = new CANSparkFlex(28, MotorType.kBrushless);
  CANSparkFlex motorRotateRight = new CANSparkFlex(29, MotorType.kBrushless);
  SparkAbsoluteEncoder throughboreRotateLeft = motorRotateLeft.getAbsoluteEncoder();
  SparkAbsoluteEncoder throughboreRotateRight = motorRotateRight.getAbsoluteEncoder();

  private static ArmRotater rotate;
  Timer rampTimer = new Timer();
  double rampTime = 0.5;
  //double powerMult = 1;
  double rotateLeftPower;
  double rotateRightPower;
  double targetRotateEncoder = throughboreRotateLeft.getPosition();
  public boolean onAngle = false;
  double currentEncoderValueLeft = 0;
  double currentEncoderValueRight = 0;

  //Ryan has changed  V  from 1 to 0.2
  final double kMaxRotateSpeedUp = 1; //.7; // up .5 down -.2
  //Ryan has changed  V  from -0.8 to -0.16
  final double kMaxRotateSpeedDown = -0.8; //-.3; //was -0.8 before hand

  static final double kLeftEncoderOffset = 0.4824;//.49942
  static final double kRightEncoderOffset = 0.49;//.4945;
  final double kMaxEncoderDifference = .01;
  double previousRotatePower = 0;
  final double kp = 9;
  final double encoderDifferenceP = 2;
  int periodicCounter = 0;
  int count = 0;

  boolean goToTransitionPos = false;
  double transitionPos = 0.17;
  double prevTarget;

  boolean runPid = true;

  /**
     * Returns the instance of ArmRotater, ensuring only one copy is made
     * @param name The name of the class requesting the ArmRotater instance
     * @return An instance of ArmRotater
  */
  public static ArmRotater getInstance(String name) {
    if(rotate == null) {
      rotate = new ArmRotater();
    }
    System.out.println(name + " aquired arm rotater");
    return rotate;
  }

  // Set all motor values
  public ArmRotater() {

    motorRotateLeft.restoreFactoryDefaults();
    motorRotateRight.restoreFactoryDefaults();

    motorRotateLeft.setInverted(true);
    motorRotateRight.setInverted(false);

    motorRotateLeft.setIdleMode(IdleMode.kCoast);
    motorRotateRight.setIdleMode(IdleMode.kCoast);

    motorRotateLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    motorRotateRight.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);

    motorRotateLeft.setSmartCurrentLimit(60);
    motorRotateRight.setSmartCurrentLimit(60);

    motorRotateLeft.burnFlash();
    motorRotateRight.burnFlash();

    rampTimer.restart();

    targetRotateEncoder = ArmConstants.armSubwoofer;
    //stop();
    //setShootInfo(currentShooterInfo);
  }

  /** Takes a value between -1 <-> 1 for both arm powers
   * @param leftPower Power for left rotation motor (not in RPM)
   * @param rightPower Power for right rotation motor (not in RPM)
  */
  public void setRotationSpeed(double leftPower, double rightPower) {
    
    motorRotateLeft.set(leftPower);
    motorRotateRight.set(rightPower);

    SmartDashboard.putNumber("rotate speed", leftPower);

  }

  // not needed maybe maybe needed
  public void setVoltage(double left, double right) {
    runPid = false;
    motorRotateLeft.set(left);
    motorRotateRight.set(right);
  }

  double previousEncoder = currentEncoderValueLeft;
  double boost = 0;
  double boostThreshold = 0.1;
  double count2 = 0;
  double minPower = 0;

  /** The PID for rotation, checks when it get to the target value
   * @param targetEncoderValue Encoder value that the arm wants to go to
   * @param currentEncoderValue Where the arm is currently at
   */
  public double rotate(double targetEncoderValue, double currentEncoderValue) {
    double error = (targetEncoderValue - currentEncoderValue);
    double currentTime = rampTimer.get();
    double rampMultiplier = currentTime < rampTime ? currentTime / rampTime : 1; 
    rampMultiplier *= rampMultiplier;
    Logger.recordOutput("Ramp", rampMultiplier);
    double power = error * kp * rampMultiplier  + boost;
    if(!onAngle) {
      if(power > 0 && power < minPower) {
        power = minPower;
      }
      else if(power < 0 && power > -minPower) {
        power = -minPower;
      }
    }

    if(power > boostThreshold) {
      //boost = .05;
    }
    else if(power < -boostThreshold) {
      //boost = -.05;
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
      // if(power - previousRotatePower > .004) {
      //   power = previousRotatePower + .004;//.004;
      // }
      // if(power - previousRotatePower < -.004) {
      //   power = previousRotatePower - .004;//.004;
      // }
    }
    previousRotatePower = power;
    previousEncoder = currentEncoderValue;
    // if (power < 0 && targetEncoderValue > 0.19) {
    //   power = MathUtil.clamp(power, -0.5, 0.5);
    // }
    return power;
  }



  // zero basing encoders left encoder has .497 offset, right has ?
  /** Get the left rotater motor in rotations */
  public double getLeftRotateEncoder() {
    return kLeftEncoderOffset - throughboreRotateLeft.getPosition() /*- kLeftEncoderOffset*/;
  }

  /** Get the right rotater motor in rotations */
  public double getRightRotateEncoder() {
    return kRightEncoderOffset - throughboreRotateRight.getPosition() /*- kRightEncoderOffset*/;
  }

  /** Set a new rotation target and run the PID again
   * @param encoderValue The new target encoder value
   */
  public int setRotationTarget(double encoderValue) {
    runPid = true;
    /*
    if (targetRotateEncoder > 0.2) {
      goToTransitionPos = true;
    }
    */

    if(encoderValue != Double.MAX_VALUE) {
      targetRotateEncoder = encoderValue;
    }
    if(encoderValue == Double.MIN_VALUE) {
      System.out.println("Set target correctly");
      targetRotateEncoder = getLeftRotateEncoder();
    }

    if (goToTransitionPos) {
      prevTarget = targetRotateEncoder;
      targetRotateEncoder = transitionPos;
    }

    manageFlags();
    rampTimer.reset();
    return periodicCounter;
  }

  public void changeSetpointBy(double value) {
    targetRotateEncoder += value;
  }


  // Call rotation methods and make sure the values are within bounds
  /** Call the PID, and make sure the current motors rotation is not out of bounds */
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

    //double errorBetween = currentEncoderValueLeft - currentEncoderValueRight;
    //rotateLeftPower -= errorBetween * encoderDifferenceP;
    //rotateRightPower += errorBetween * encoderDifferenceP;

    if(Math.abs(currentEncoderValueLeft - currentEncoderValueRight) > kMaxEncoderDifference) {
      rotateLeftPower = 0;
      rotateRightPower = 0;
      System.out.println("rotate encoder misaligned");
    }
    Logger.recordOutput("LeftArmPower", rotateLeftPower);
    Logger.recordOutput("RightArmPower", rotateRightPower);
    setRotationSpeed(rotateLeftPower, rotateRightPower);

    if (currentEncoderValueLeft < 0.2 && goToTransitionPos) {
      goToTransitionPos = false;
      targetRotateEncoder = prevTarget;
    }
  }

  // Check if it is on the correct angle
  /** Check if the encoder value is at the target and then set on angle to true */
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

    // Run PID, call the methods that manage everything
    if (runPid) {
      manageRotate();
      manageFlags();
    }
    
    periodicCounter++;
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("left throughbore position", currentEncoderValueLeft);
    //SmartDashboard.putNumber("Current Left Throughbore", leftRotateThroughbore.getPosition());
    SmartDashboard.putNumber("right throughbore position", currentEncoderValueRight);
    SmartDashboard.putNumber("Target left Encoder", targetRotateEncoder);
    SmartDashboard.putBoolean("On angle", onAngle);

    Logger.recordOutput("Arm/Left Encoder", getLeftRotateEncoder());
    Logger.recordOutput("Arm/Right Encoder", getRightRotateEncoder());
    Logger.recordOutput("Arm/Encoder Target", targetRotateEncoder);
    Logger.recordOutput("OnAngle", onAngle);
    //System.out.println(currentShooterInfo.intakeSpeed + " INTAKE SPEED");
}

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
