// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class TimeStampCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Shooter m_subsystem;
  Timer timer = new Timer();

  /**
   * Creates a new ExampleCommand.
   *
   * @param shooter The subsystem used by this command.
   */
  public TimeStampCommand(Shooter shooter) {
    m_subsystem = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    timer.reset();
    timer.start();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(timer.get() + ": Init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(timer.get() + ": Exec");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println(timer.get() + ": IsFin");

    return false;
  }
}
