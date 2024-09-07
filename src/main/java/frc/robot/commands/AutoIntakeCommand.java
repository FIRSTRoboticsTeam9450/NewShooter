package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.LaunchPosition;
import frc.robot.subsystems.Shooter;

/** Automates the whole process of intaking a note. Moves the arm down, runs the intake, moves the arm up once a note is detected, and prepares the note to be fired */
public class AutoIntakeCommand extends Command {

    Command goToIntakeAngle = new SetLauncherAngleCommand(LaunchPosition.INTAKE);
    Command goToStorageAngle = new SetLauncherAngleCommand(LaunchPosition.SUBWOOFER);
    Command runIntake = new IntakeNoteCommand();
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
