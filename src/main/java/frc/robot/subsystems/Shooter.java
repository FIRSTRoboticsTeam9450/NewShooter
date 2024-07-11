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
  public boolean noteIn = false;
  public boolean noteShot = false;
  double normalUpper = 0;
  double normalLower = 0;
  double upperSpeed = 5676;
  double lowerSpeed = 6784;
  double power = 0;
  double upperPower;
  double lowerPower;
  double previousUpper = 0;
  double previousLower = 0;
  public boolean shooterMotorsOn = false;
  public boolean onAngle = false;
  ShootInfo stop = new ShootInfo(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ShootPosition.STOP);

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

    leftRotateMotor.setInverted(false);
    rightRotateMotor.setInverted(true);

    upperShooterMotor.setIdleMode(IdleMode.kCoast);
    lowerShooterMotor.setIdleMode(IdleMode.kCoast);

    currentShooterInfo = stop.copy();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command cancel() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          this.cancel();
          /* one-time action goes here */
        });
  }
  
  public void initializeShoot() {
    started = false;
    intakeOn = false;
    noteIn = false;
    normalUpper = 0;
    normalLower = 0;
    previousLower = 0;
    previousUpper = 0;
    shooterMotorsOn = false;
    count = 0;
    timer.reset();

  }


  public boolean runShooterMotors(ShootPosition type, double upperPower, double lowerPower) {
    SmartDashboard.putNumber("Upper Speed", upperEncoder.getVelocity());
    SmartDashboard.putNumber("Lower Speed", lowerEncoder.getVelocity());

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
    normalUpper = upperEncoder.getVelocity();
    normalLower = lowerEncoder.getVelocity();
    if(Math.abs(previousLower - normalLower) < 1 && Math.abs(previousUpper - normalUpper) < 1) {
      count++;
      if(count == 3) {
        shooterMotorsOn = true;
        return true;
      }
    }
    previousLower = normalLower;
    previousUpper = normalUpper;

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

  public boolean startIntake() {
    if(!intakeOn) {
      setIntakeSpeed(-.5);
    }
    if(intakeOn){
      
      if(upperEncoder.getVelocity() - normalUpper < -600 || lowerEncoder.getVelocity() - normalLower < -600) {
        noteIn = true;
      }

      if(noteIn) {
        if(upperEncoder.getVelocity() >= normalUpper || lowerEncoder.getVelocity() >= normalUpper) {
            return true;
        }
      }
    }
    return false;
  }
  public void setShooterMotorSpeed(double upper, double lower) {
    upperShooterMotor.set(upper);
    lowerShooterMotor.set(lower);
  }

  public void setRotationSpeed(double power, double encoderValue) {
    if(encoderValue < leftRotateThroughbore.getPosition()) {
      power *= -1;
    }
    
    System.out.println("Power:" + power);
    leftRotateMotor.set(power);
    rightRotateMotor.set(power);

  }
  public boolean rotateTil(double encoderValue) {
    if(Math.abs(encoderValue - leftRotateThroughbore.getPosition()) < .001) {
      shooter.setRotationSpeed(0, leftRotateThroughbore.getPosition());
      return true;
    }
    else if(power < 0 && encoderValue > leftRotateThroughbore.getPosition()) {
      shooter.setRotationSpeed(0, leftRotateThroughbore.getPosition());
      return true;
    } 
    else if(power > 0 && encoderValue < leftRotateThroughbore.getPosition()) {
      shooter.setRotationSpeed(0, leftRotateThroughbore.getPosition());
      return true;
    }


    //System.out.println("Worked");
    return false;
  }
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

  public void setShootInfo(ShootInfo info) {
    if(!currentShooterInfo.isEqual(info)) {
      System.out.println(info.type);
      System.out.println(info.intakeSpeed);
      System.out.println(info.rotationSpeed);
      currentShooterInfo = info.copy();
      currentShooterInfo.setNew(true);
    }
    else {
      System.out.println("Already going \n type:" + info.type);
    }
  }

  @Override
  public void periodic() {
    if(!currentShooterInfo.type.equals(ShootPosition.STOP)) {
      if(currentShooterInfo.isNew()) {
        initializeShoot();
        setIntakeSpeed(currentShooterInfo.intakeSpeed);
        setShooterMotorSpeed(currentShooterInfo.upperPower, currentShooterInfo.lowerPower);
        setRotationSpeed(currentShooterInfo.rotationSpeed, currentShooterInfo.encoderValue);
        currentShooterInfo.setNew(false);
        System.out.println(currentShooterInfo.isNew());
      }

      
      onAngle = rotateTil(currentShooterInfo.encoderValue);
      System.out.println("ON ANGLE:"+onAngle);
      shooterMotorsOn = runShooterMotors(currentShooterInfo.type, currentShooterInfo.upperPower, currentShooterInfo.lowerPower);
      
      noteShot = startIntake();
    }

    
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
