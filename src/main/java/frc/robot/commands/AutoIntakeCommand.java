package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.RobotConstants;
import frc.robot.subsystems.LaunchPosition;

/** Automates the whole process of intaking a note. Moves the arm down, runs the intake, moves the arm up once a note is detected, and prepares the note to be fired */
public class AutoIntakeCommand extends Command {

    double launchAngle;

    Command toRun;
    Command stopWheels = new SetLauncherSpeedCommand(0, 0);

    public AutoIntakeCommand() {
        this(RobotConstants.armSubwoofer);
    }

    public AutoIntakeCommand(double launchAngle) {
        Command goToIntakeAngle = new SetLauncherAngleCommand(LaunchPosition.INTAKE);
        Command goToStorageAngle = new SetLauncherAngleCommand(launchAngle);
        Command runIntake = new IntakeNoteCommand();
        Command wait = new WaitCommand(0.5);
        Command processNote = new ProcNoteCommand();

        toRun = new ParallelCommandGroup(
            goToIntakeAngle,
            new SequentialCommandGroup(
                runIntake,
                goToStorageAngle,
                wait,
                processNote
            )
        );
    }

    @Override
    public void initialize() {
        toRun.schedule();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            stopWheels.schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return toRun.isFinished();
    }
}
