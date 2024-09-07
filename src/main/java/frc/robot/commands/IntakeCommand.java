package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ShootPosition;
import frc.robot.subsystems.Shooter;

public class IntakeCommand extends Command {

    Command goToIntakeAngle = new SetAngleCommandTest2(ShootPosition.INTAKE);
    Command goToStorageAngle = new SetAngleCommandTest2(ShootPosition.SUBWOOFER);
    Command runIntake = new AutoIntakeCommand();
    Command wait = new WaitCommand(0.5);
    Command processNote = new ProcNoteCommand();

    Command toRun = new ParallelCommandGroup(
        goToIntakeAngle,
        new SequentialCommandGroup(
            runIntake,
            goToStorageAngle,
            wait,
            processNote
        )
    );

    @Override
    public void initialize() {
        toRun.schedule();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return toRun.isFinished();
    }
}
