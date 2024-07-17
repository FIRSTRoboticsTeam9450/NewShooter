package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.InfoParams;
import frc.robot.subsystems.ShootInfo;
import frc.robot.subsystems.Shooter;

public class FireNote extends Command{
    Shooter shooter = Shooter.getInstance("Fire");
    public FireNote() {
    }

  // Called when the command is initially scheduled.
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  public void execute() {
    if(SetAngleCommand.fire) {
        ShootInfo fire = new ShootInfo(InfoParams.IGNORE);
        fire.intakeSpeed = 1;
        shooter.setShootInfo(fire);
        SetAngleCommand.fired = true;
        SetAngleCommand.fire = false;
    }
  }

  // Called once the command ends or is interrupted.
  public void end() {
    SetAngleCommand.fire = false;
    SetAngleCommand.fired = true;
  }

  // Returns true when the command should end.
  public boolean isFinished() {
    if(SetAngleCommand.fired) {
        return true;
    }
    return false;
  }
}
