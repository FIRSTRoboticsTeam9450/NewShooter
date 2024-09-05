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

public class Shooter extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  CANSparkFlex motorIntake = new CANSparkFlex(32, MotorType.kBrushless);
  CANSparkFlex motorShooterUpper = new CANSparkFlex(30, MotorType.kBrushless);
  CANSparkFlex motorShooterLower = new CANSparkFlex(31, MotorType.kBrushless);
  CANSparkFlex motorRotateLeft = new CANSparkFlex(28, MotorType.kBrushless);
  CANSparkFlex motorRotateRight = new CANSparkFlex(29, MotorType.kBrushless);
  RelativeEncoder encoderShooterUpper = motorShooterUpper.getEncoder();
  RelativeEncoder encoderShooterLower = motorShooterLower.getEncoder();
  SparkAbsoluteEncoder throughboreRotateLeft = motorRotateLeft.getAbsoluteEncoder();
  SparkAbsoluteEncoder throughboreRotateRight = motorRotateRight.getAbsoluteEncoder();

  private LaserCan laser;
  private LaserCan.Measurement measurement; 
  MedianFilter medianDistance = new MedianFilter(3);
  boolean laserIsDead = false;
  double laserDistance;

  private static Shooter shooter;
  Timer timer = new Timer();
  boolean intakeOn = false;
  public boolean noteIn = false;
  public boolean noteShot = false;
  double normalUpper = 0;
  double normalLower = 0;
  double encoderValueUpper = 0;
  double encoderValueLower = 0;
  //double powerMult = 1;
  double rotateLeftPower;
  double rotateRightPower;
  double encoderUpperPrevious = 0;
  double encoderLowerPrevious = 0;
  public boolean shooterMotorsOn = false;
  public boolean onAngle = false;
  public boolean checkRight = false;
  double currentEncoderValueLeft = 0;
  double currentEncoderValueRight = 0;
  final double kMaxRotateSpeedUp = 0.1; //.7; // up .5 down -.2
  final double kMaxRotateSpeedDown = -0.1; //-.3;
  static final double kLeftEncoderOffset = 0.51;//.49942
  static final double kRightEncoderOffset = 0.516;//.4945;
  final double kMaxEncoderDifference = .02;
  double previousRotatePower = 0;
  boolean oneTime = true;
  final double kp = 9;
  double previousIntakeSpeed = 0;
  double previousUpperSpeed = 0;
  double previousLowerSpeed = 0;
  int periodicCounter = 0;
  boolean intakeSpeedFinal = false;
  ShootInfo stop = new ShootInfo(InfoParams.USE_CURRENT, 0.0, 0.0, 0.0, 0.0);
  int count = 0;

  ShootInfo currentShooterInfo = stop.copy();
  public static Shooter getInstance(String name) {
    if(shooter == null) {
      shooter = new Shooter();
    }
    System.out.println(name + " aquired Shooter");
    return shooter;
  }
  public Shooter() {
    motorShooterUpper.restoreFactoryDefaults();
    motorShooterLower.restoreFactoryDefaults();

    motorRotateLeft.restoreFactoryDefaults();
    motorRotateRight.restoreFactoryDefaults();

    motorIntake.restoreFactoryDefaults();
    motorIntake.setInverted(true);
    motorIntake.setIdleMode(IdleMode.kBrake);

    motorShooterUpper.setInverted(false);
    motorShooterLower.setInverted(true);

    motorRotateLeft.setInverted(true);
    motorRotateRight.setInverted(false);
    // leftRotateMotor.setIdleMode(IdleMode.kCoast);
    // rightRotateMotor.setIdleMode(IdleMode.kCoast);

    motorShooterUpper.setIdleMode(IdleMode.kCoast);
    motorShooterLower.setIdleMode(IdleMode.kCoast);

    motorRotateLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    motorRotateRight.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);

    //stop();
    setShootInfo(currentShooterInfo);
    laser = new LaserCan(40);
    try {
      laser.setRangingMode(LaserCan.RangingMode.LONG);
      laser.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 4, 4));
      laser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch(ConfigurationFailedException e) {
      System.out.println("Laser configuration failed " + e);
    }
    measurement = laser.getMeasurement();
  }
  
  public void initializeShoot() {
    intakeOn = false;
    noteIn = false;
    normalUpper = 0;
    normalLower = 0;
    encoderLowerPrevious = 0;
    encoderUpperPrevious = 0;
    onAngle = false;
    checkRight = false;
    shooterMotorsOn = false;
    noteShot = false;
    oneTime = true;
    count = 0;
    previousRotatePower = 0;
    timer.reset();

  }


  public boolean checkShooterMotors(double upperPower, double lowerPower) {

    if(upperPower != 0 && lowerPower != 0) {
      normalUpper = encoderValueUpper;
      normalLower = encoderValueLower;
      if(Math.abs(encoderLowerPrevious - encoderValueLower) < 1 && Math.abs(encoderUpperPrevious - encoderValueUpper) < 1) {
        count++;
        if(count >= 3) {
          return true;
        }
      }
      else {
        count = 0;
      }
      encoderLowerPrevious = encoderValueLower;
      encoderUpperPrevious = encoderValueUpper;
    }

    return false;
  }

  public void setShooterMotorSpeed(double upper, double lower) {
    motorShooterUpper.set(upper);
    motorShooterLower.set(lower);
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

  public void setIntakeSpeed(double power) {
    motorIntake.set(power);
  }

  public void resetMotors() {
    setIntakeSpeed(0);
    setShooterMotorSpeed(0, 0);
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

  public double getLaserDistance() {
    try {
      measurement = laser.getMeasurement();
      if(measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
        laserIsDead = false;
        return medianDistance.calculate(measurement.distance_mm);
      }
    } catch (Exception e) {
      // TODO: handle exception
      laserIsDead = true;
      e.printStackTrace();
    }
    return 100000;
  }

  // zero basing encoders left encoder has .497 offset, right has ?

  public double getLeftRotateEncoder() {
    return kLeftEncoderOffset - throughboreRotateLeft.getPosition() /*- kLeftEncoderOffset*/;
  }

  public double getRightRotateEncoder() {
    return kRightEncoderOffset - throughboreRotateRight.getPosition() /*- kRightEncoderOffset*/;
  }

  public int setShootInfo(ShootInfo info) {
    System.out.println("In setShootInfo");
    System.out.print(info.intakeSpeed + " ");
    System.out.print(info.upperShooterPower + " ");
    System.out.print(info.lowerShooterPower + " ");
    System.out.print(info.rotationSpeed + " ");
    System.out.println(info.targetRotateEncoder);
    if(info.intakeSpeed != Double.MAX_VALUE) {
      currentShooterInfo.intakeSpeed = info.intakeSpeed;
    }
    if(info.upperShooterPower != Double.MAX_VALUE) {
      currentShooterInfo.upperShooterPower = info.upperShooterPower;
    }
    if(info.lowerShooterPower != Double.MAX_VALUE) {
      currentShooterInfo.lowerShooterPower = info.lowerShooterPower;
    }
    if(info.rotationSpeed != Double.MAX_VALUE) {
      currentShooterInfo.rotationSpeed = info.rotationSpeed;
    }
    if(info.targetRotateEncoder != Double.MAX_VALUE) {
      currentShooterInfo.targetRotateEncoder = info.targetRotateEncoder;
    }
    if(info.targetRotateEncoder == Double.MIN_VALUE) {
      System.out.println("Set target correctly");
      currentShooterInfo.targetRotateEncoder = getLeftRotateEncoder();
    }
    currentShooterInfo.setNew(true);
    manageFlags();

    return periodicCounter;
  }

  public void stop() {
    setShootInfo(stop);
  }

  private void manageRotate() {
    currentEncoderValueLeft = getLeftRotateEncoder();
    currentEncoderValueRight = getRightRotateEncoder();
    if (currentShooterInfo.targetRotateEncoder > (0.708 - 0.497)) {
      currentShooterInfo.targetRotateEncoder = 0.708 - 0.497;
    }
    if (currentShooterInfo.targetRotateEncoder < 0) {
      currentShooterInfo.targetRotateEncoder = 0;
    }

    rotateLeftPower = rotate(currentShooterInfo.targetRotateEncoder, currentEncoderValueLeft);
    rotateRightPower = rotate(currentShooterInfo.targetRotateEncoder, currentEncoderValueRight);
    if(Math.abs(currentEncoderValueLeft - currentEncoderValueRight) > kMaxEncoderDifference) {
      rotateLeftPower = 0;
      rotateRightPower = 0;
      System.out.println("rotate encoder misaligned");
    }
    setRotationSpeed(rotateLeftPower, rotateRightPower);
  }

  private void manageFlags() {

    if(Math.abs(currentShooterInfo.targetRotateEncoder - currentEncoderValueLeft) < .001) {
      onAngle = true;
    }
    else if(currentEncoderValueLeft > currentShooterInfo.targetRotateEncoder && previousEncoder < currentShooterInfo.targetRotateEncoder) {
      onAngle = true;
    }
    else if(currentEncoderValueLeft < currentShooterInfo.targetRotateEncoder && previousEncoder > currentShooterInfo.targetRotateEncoder) {
      onAngle = true;
    }
    else {
      onAngle = false;
    }
    
    if(currentShooterInfo.upperShooterPower == 0) {
      shooterMotorsOn = false;
    }
    // to do: change to velocity PID
    shooterMotorsOn = checkShooterMotors(currentShooterInfo.upperShooterPower, currentShooterInfo.lowerShooterPower);
    
    if(laserDistance <= 60) {
      noteIn = true;
    }
    else {
      noteIn = false;
    }
  }

  public int getPeriodicCounter() {
    return periodicCounter;
  }

  @Override
  public void periodic() {
    laserDistance = getLaserDistance(); // 60 mm is correct for intake

    manageRotate();

    encoderValueLower = encoderShooterLower.getVelocity();
    encoderValueUpper = encoderShooterUpper.getVelocity();
    if(previousUpperSpeed != currentShooterInfo.upperShooterPower || previousLowerSpeed != currentShooterInfo.lowerShooterPower) {
      setShooterMotorSpeed(currentShooterInfo.upperShooterPower, currentShooterInfo.lowerShooterPower);
      previousUpperSpeed = currentShooterInfo.upperShooterPower;
      previousLowerSpeed = currentShooterInfo.lowerShooterPower;
    }

    if(previousIntakeSpeed != currentShooterInfo.intakeSpeed) {
      setIntakeSpeed(currentShooterInfo.intakeSpeed);
      previousIntakeSpeed = currentShooterInfo.intakeSpeed;
    }

    manageFlags();
    periodicCounter++;
    SmartDashboard.putBoolean("Note in", noteIn);
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("left throughbore position", currentEncoderValueLeft);
    //SmartDashboard.putNumber("Current Left Throughbore", leftRotateThroughbore.getPosition());
    SmartDashboard.putNumber("right throughbore position", currentEncoderValueRight);
    SmartDashboard.putNumber("Target left Encoder", currentShooterInfo.targetRotateEncoder);
    SmartDashboard.putBoolean("On angle", onAngle);
    SmartDashboard.putBoolean("Shooter motors ready", shooterMotorsOn);
    SmartDashboard.putNumber("Intake speed", currentShooterInfo.intakeSpeed);
    SmartDashboard.putBoolean("Note flown", noteShot);
    SmartDashboard.putBoolean("Note In", noteIn);
    SmartDashboard.putNumber("Upper encoder", encoderValueUpper);
    SmartDashboard.putNumber("Lower encoder", encoderValueLower);
    //System.out.println(currentShooterInfo.intakeSpeed + " INTAKE SPEED");
}

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
