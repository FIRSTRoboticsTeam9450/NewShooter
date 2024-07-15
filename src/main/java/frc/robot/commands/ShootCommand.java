// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ShootInfo;
import frc.robot.subsystems.ShootPosition;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ShootCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Shooter shooter = Shooter.getInstance("ShootCommand");
  ShootPosition shotType;
  ShootInfo info;
  ShootInfo preSetInfo;
  boolean runOnceRotate = true;
  boolean runOnceShoot = true;
  int previousPeriodicTime = 0;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShootCommand(ShootPosition type, ShootInfo info) {
    shotType = type;
    preSetInfo = info.copy();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    info = preSetInfo.copy();
    info.intakeSpeed = 0;
    shooter.initializeShoot();
    shooter.setShootInfo(info);
    runOnceRotate = true;
    runOnceShoot = true;
  }

  // Called every time the scheduler runs while the command is scheduled.

  //boolean finish = false;
  @Override
  public void execute() {
    // finish = shooter.runShooterMotors(shotType, 0, 0);
    
    if(shooter.onAngle) {
      if(runOnceRotate) {
        info.rotationSpeed = 0;
        shooter.setShootInfo(info);
        runOnceRotate = false;
      }
    }
    if(shooter.onAngle && shooter.shooterMotorsOn && shooter.noteIn) {
      System.out.println("YIPPPEEEEE");
      if(runOnceShoot) {
        info.intakeSpeed = preSetInfo.intakeSpeed;
        //info.upperPower = 0;
        shooter.setShootInfo(info);
        
        runOnceShoot = false;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooter.noteShot;
    
    // if(info.type.equals(ShootPosition.STOP)) {
    //   return true;
    // }
    // return false;

  }
}
