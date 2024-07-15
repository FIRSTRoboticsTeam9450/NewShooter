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
  CANSparkFlex intakeMotor = new CANSparkFlex(32, MotorType.kBrushless);
  CANSparkFlex upperShooterMotor = new CANSparkFlex(30, MotorType.kBrushless);
  CANSparkFlex lowerShooterMotor = new CANSparkFlex(31, MotorType.kBrushless);
  CANSparkMax leftRotateMotor = new CANSparkMax(28, MotorType.kBrushless);
  CANSparkMax rightRotateMotor = new CANSparkMax(29, MotorType.kBrushless);
  RelativeEncoder upperEncoder = upperShooterMotor.getEncoder();
  RelativeEncoder lowerEncoder = lowerShooterMotor.getEncoder();
  SparkAbsoluteEncoder leftRotateThroughbore = leftRotateMotor.getAbsoluteEncoder();
  SparkAbsoluteEncoder rightRotateThroughbore = rightRotateMotor.getAbsoluteEncoder();

  private LaserCan laser;
  private LaserCan.Measurement measurement; 
  MedianFilter medianDistance = new MedianFilter(3);
  boolean laserIsDead = false;
  double laserDistance;

  private static Shooter shooter;
  Timer timer = new Timer();
  boolean started = false;
  boolean intakeOn = false;
  public boolean noteIn = false;
  public boolean noteShot = false;
  double normalUpper = 0;
  double normalLower = 0;
  double upperEncoderValue = 0;
  double lowerEncoderValue = 0;
  //double powerMult = 1;
  double leftPower;
  double rightPower;
  double upperPower;
  double lowerPower;
  double previousUpper = 0;
  double previousLower = 0;
  public boolean shooterMotorsOn = false;
  public boolean onAngle = false;
  public boolean checkRight = false;
  double currentEncoderValueLeft = 0;
  double currentEncoderValueRight = 0;
  final double kMaxRotateSpeedUp = .5; // up .5 down -.2
  final double kMaxRotateSpeedDown = -.3;
  static final double kLeftEncoderOffset = .497;
  final double kRightEncoderOffset = .497;
  final double kMaxEncoderDifference = .02;
  double previousRotatePower = 0;
  boolean oneTime = true;
  final double kp = 3;
  double previousIntakeSpeed = 0;
  double previousUpperSpeed = 0;
  double previousLowerSpeed = 0;
  int periodicCounter = 0;
  boolean intakeSpeedFinal = false;
  ShootInfo stop = new ShootInfo(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  int count = 0;

  ShootInfo currentShooterInfo;
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

    leftRotateMotor.setInverted(true);
    rightRotateMotor.setInverted(false);

    upperShooterMotor.setIdleMode(IdleMode.kCoast);
    lowerShooterMotor.setIdleMode(IdleMode.kCoast);

    leftRotateMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    rightRotateMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);

    currentShooterInfo = stop.copy();

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

  /**
   * Example command factory method.
   *
   * @return a command
   */
  // public Command cancel() {
  //   // Inline construction of command goes here.
  //   // Subsystem::RunOnce implicitly requires `this` subsystem.
  //   return runOnce(
  //       () -> {
  //         this.cancel();
  //         /* one-time action goes here */
  //       });
  // }
  
  public void initializeShoot() {
    started = false;
    intakeOn = false;
    noteIn = false;
    normalUpper = 0;
    normalLower = 0;
    previousLower = 0;
    previousUpper = 0;
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
    SmartDashboard.putNumber("Upper encoder", upperEncoderValue);
    SmartDashboard.putNumber("Lower encoder", lowerEncoderValue);

    // switch(type) {
    //   case AMP:
    //     upperPower = .1;
    //     lowerPower = .25;
    //     break;
    //   case NORMAL:
    //     upperPower = 1;
    //     lowerPower = 1;
    //     break;
      
    // }
    normalUpper = upperEncoderValue;
    normalLower = lowerEncoderValue;
    if(Math.abs(previousLower - lowerEncoderValue) < 1 && Math.abs(previousUpper - upperEncoderValue) < 1) {
      count++;
      if(count >= 3) {
        return true;
      }
    }
    previousLower = lowerEncoderValue;
    previousUpper = upperEncoderValue;

    // if(!started) {
    //   timer.reset();
    //   timer.start();
    //   started = true;
    // }
    // if(intakeOn) {
    //   if(upperEncoder.getVelocity() - normalUpper < -600 || lowerEncoder.getVelocity() - normalLower < -600) {
    //     noteIn = true;
    //   }

    //   if(noteIn) {
    //     if(upperEncoder.getVelocity() >= normalUpper || lowerEncoder.getVelocity() >= normalUpper) {
    //       return true;
    //     }
    //   }
    // }
    // if(timer.hasElapsed(1)) {
    //   timer.stop();
    //   intakeOn = true;
    // }
    return false;
  }

  public boolean startIntake(double power) {
  
    // if(upperEncoderValue - normalUpper < -600 || lowerEncoderValue - normalLower < -600) {
    //   noteIn = true;
    // }

    // if(noteIn) {
    //   if(upperEncoderValue >= normalUpper || lowerEncoderValue >= normalUpper) {
    //       return true;
    //   }
    // }
    return false;
  }
  public void setShooterMotorSpeed(double upper, double lower) {
    upperShooterMotor.set(upper);
    lowerShooterMotor.set(lower);
  }

    
  // public double changeDirection(double encoderValue, double currentValue, double power) {
  //   if(encoderValue > currentValue) {
  //     return -Math.abs(power);
  //   }
  //   else{
  //     return Math.abs(power);
  //   }
  // }
  public void setRotationSpeed(double leftPower, double rightPower, double encoderValue) {
    
    // leftPower = changeDirection(encoderValue, currentEncoderValue, leftPower);
    // rightPower = changeDirection(encoderValue, currentEncoderValue, rightPower);
    //System.out.println("Power:" + leftPower);
    leftRotateMotor.set(leftPower);
    rightRotateMotor.set(rightPower);

    SmartDashboard.putNumber("rotate speed", leftPower);

  }

  public double rotate(double targetEncoderValue, double currentEncoderValue) {
    double error = (targetEncoderValue - currentEncoderValue);
    double power = (targetEncoderValue - currentEncoderValue) * kp;

    if(error >= 0) {
      power = Math.min(kMaxRotateSpeedUp, power); // Limiting power when going up
    }
    else{
      power = Math.max(kMaxRotateSpeedDown, power); // Limiting power when going down
    }
    if(Math.signum(previousRotatePower) != Math.signum(power)) {
      previousRotatePower = 0;
    }
    if(power - previousRotatePower > .01) {
      power = previousRotatePower + .001;
    }
    else if(power - previousRotatePower < -.01) {
      power = previousRotatePower - .001;
    }

    previousRotatePower = power;
    return power;
  }

  // public boolean rotateTil(double encoderValue, double currentValue) {
    
  //   if(Math.abs(encoderValue - currentValue) < .014) {
  //     shooter.setRotationSpeed(0, 0, currentValue);
  //     checkRight = true;
  //     System.out.println("GOT HERE" + encoderValue + ":" + currentValue);
  //   }
    // else if(currentShooterInfo.rotationSpeed < 0 && encoderValue > currentValue) {
    //   shooter.setRotationSpeed(0, 0, currentValue);
    //   checkRight = true;
    //   System.out.println("GOT HERE it was greater"+ encoderValue + ":" + currentValue);
    // } 
    // else if(currentShooterInfo.rotationSpeed > 0 && encoderValue < currentValue) {
    //   shooter.setRotationSpeed(0, 0, currentValue);
    //   checkRight = true;
    //   System.out.println("GOT HERE less than"+ encoderValue + ":" + currentValue);
    // }
    // return checkRight;
    // currentValue = rightRotateThroughbore.getPosition();
    // if(checkRight) {
    //   if(Math.abs(encoderValue - currentValue) < .001) {
    //     shooter.setRotationSpeed(0, 0, rightRotateThroughbore.getPosition());
    //     System.out.println("GOT HERE 2X");
    //     return true;
    //   }
    //   else if(currentShooterInfo.rotationSpeed < 0 && encoderValue > rightRotateThroughbore.getPosition()) {
    //     shooter.setRotationSpeed(0, 0, rightRotateThroughbore.getPosition());
    //     System.out.println("GOT HERE 2X");
    //     return true;
    //   } 
    //   else if(currentShooterInfo.rotationSpeed > 0 && encoderValue < leftRotateThroughbore.getPosition()) {
    //     shooter.setRotationSpeed(0, 0, rightRotateThroughbore.getPosition());
    //     System.out.println("GOT HERE 2X");
    //     return true;
    //   }
    //   System.out.println(currentShooterInfo.encoderValue + " PLEASEEEEEEEEEEEEEEE " + leftRotateThroughbore.getPosition());
    // }
    //System.out.println("Worked");
    // return false;
  // }
  public void setIntakeSpeed(double power) {
    intakeMotor.set(power);
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
    return leftRotateThroughbore.getPosition() - kLeftEncoderOffset;
  }

  public double getRightRotateEncoder() {
    return rightRotateThroughbore.getPosition() - kRightEncoderOffset;
  }

  public int setShootInfo(ShootInfo info) {
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
    currentShooterInfo.setNew(true);
    // }
    // else {
    //   System.out.println("Already going \n type:" + info.type);
    // }
    return periodicCounter;
  }

  public void stop() {
    stop.targetRotateEncoder = getLeftRotateEncoder();
    setShootInfo(stop);
  }

  private void manageRotate() {
    currentEncoderValueLeft = getLeftRotateEncoder();
    currentEncoderValueRight = getRightRotateEncoder();
    leftPower = rotate(currentShooterInfo.targetRotateEncoder, currentEncoderValueLeft);
    rightPower = rotate(currentShooterInfo.targetRotateEncoder, currentEncoderValueRight);
    if(Math.abs(currentEncoderValueLeft - currentEncoderValueRight) > kMaxEncoderDifference) {
      leftPower = 0;
      rightPower = 0;
      System.out.println("rotate encoder misaligned");
    }
    setRotationSpeed(leftPower, rightPower, currentShooterInfo.targetRotateEncoder);
  }

  private void manageFlags() {

    if(Math.abs(currentShooterInfo.targetRotateEncoder - currentEncoderValueLeft) < .005) {
      onAngle = true;
    }
    else {
      onAngle = false;
    }
    if(!shooterMotorsOn)
    {
      // to do: change to velocity PID
      shooterMotorsOn = checkShooterMotors(currentShooterInfo.upperShooterPower, currentShooterInfo.lowerShooterPower);
    }
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

    lowerEncoderValue = lowerEncoder.getVelocity();
    upperEncoderValue = upperEncoder.getVelocity();
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
    //System.out.println(currentShooterInfo.intakeSpeed + "_____________ INTAKE SPEED");
    

    //onAngle = rotateTil(currentShooterInfo.encoderValue, currentEncoderValue);
    //System.out.println("ON ANGLE:"+onAngle);
    
    
    // else {
    //   if(currentShooterInfo.isNew()) {
    //     setIntakeSpeed(currentShooterInfo.intakeSpeed);
    //     setShooterMotorSpeed(currentShooterInfo.upperPower, currentShooterInfo.lowerPower);
    //     //setRotationSpeed(currentShooterInfo.rotationSpeed, currentShooterInfo.rotationSpeed, currentShooterInfo.encoderValue);
    //     currentShooterInfo.setNew(false);
    //   }
    // }

    
    // if(!currentShooterInfo.type.equals(ShootPosition.STOP)){
    //   System.out.println("worked");
    //     setRotationSpeed(-.05, currentShooterInfo.encoderValue);
    //     initializeShoot();
        
        
    //     currentShooterInfo.setNew(false);
    // }

    // if(!currentShooterInfo.type.equals(ShootPosition.INTAKE)) {
    //   if(!shooterMotorsOn) {
    //       runShooterMotors(currentShooterInfo.type, currentShooterInfo.upperPower, currentShooterInfo.lowerPower);
          
    //       onAngle = rotateTil(currentShooterInfo.encoderValue);
    //       if(onAngle) { // && shooterMotorsOn
    //         if(startIntake()) {
    //             resetMotors();
    //             currentShooterInfo = stop.copy();
    //             System.out.println("yippee");
    //         }
    //       }
    //     }
    // }
    // else {
    //   if(onAngle) { // && shooterMotorsOn
    //     if(startIntake()) {
    //         resetMotors();
    //         currentShooterInfo = stop.copy();
    //         System.out.println("yippee");
    //     }
    //   }
    // }

    //System.out.println("Worked");
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("left throughbore position", currentEncoderValueLeft);
    SmartDashboard.putNumber("Current Left Throughbore", leftRotateThroughbore.getPosition());
    SmartDashboard.putNumber("left throughbore velocity", leftRotateThroughbore.getVelocity());
    SmartDashboard.putNumber("right throughbore position", rightRotateThroughbore.getPosition());
    SmartDashboard.putNumber("right throughbore velocity", rightRotateThroughbore.getVelocity());
    SmartDashboard.putNumber("Target left Encoder", currentShooterInfo.targetRotateEncoder);
    SmartDashboard.putBoolean("On angle", onAngle);
    SmartDashboard.putBoolean("Shooter motors ready", shooterMotorsOn);
    SmartDashboard.putNumber("Intake speed", currentShooterInfo.intakeSpeed);
    SmartDashboard.putBoolean("Note flown", noteShot);
    //System.out.println(currentShooterInfo.intakeSpeed + " INTAKE SPEED");
}

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
