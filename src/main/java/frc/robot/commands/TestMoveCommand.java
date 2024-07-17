package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class TestMoveCommand extends Command{
    Shooter shooter= Shooter.getInstance("TestMoveCommand");
    //Tester tester = Tester.getInstance();

    public TestMoveCommand(){

    }

    /** The initial subroutine of a command. Called once when the command is initially scheduled. */
    public void initialize() {

    }

    /** The main body of a command. Called repeatedly while the command is scheduled. */
    public void execute() {

    }

    /**
     * The action to take when the command ends. Called when either the command finishes normally, or
     * when it interrupted/canceled.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    public void end(boolean interrupted) {

    }

    /**
     * Whether the command has finished. Once a command finishes, the scheduler will call its end()
     * method and un-schedule it.
     *
     * @return whether the command has finished.
     */
    public boolean isFinished() {
        return false;
    }

}
