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
  private final Shooter shooter;
  ShootPosition shotType;
  ShootInfo info;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShootCommand(Shooter subsystem, ShootPosition type, ShootInfo info) {
    shooter = subsystem;
    shotType = type;
    this.info = info.copy();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setShootInfo(info);
  }

  // Called every time the scheduler runs while the command is scheduled.

  //boolean finish = false;
  @Override
  public void execute() {
    // finish = shooter.runShooterMotors(shotType, 0, 0);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // shooter.resetMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(info.type.equals(ShootPosition.STOP)) {
      return true;
    }
    return false;

  }
}
