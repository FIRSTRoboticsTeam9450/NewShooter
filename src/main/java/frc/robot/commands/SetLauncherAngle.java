package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.InfoParams;
import frc.robot.subsystems.ShootInfo;
import frc.robot.subsystems.Shooter;

public class SetLauncherAngle extends Command {

    double encoderValue;
    Shooter shooter = Shooter.getInstance("SetLauncherAngle");

    public SetLauncherAngle(double encoderValue) {
        this.encoderValue = encoderValue;
    }

    @Override
    public void initialize() {
        ShootInfo info = new ShootInfo(InfoParams.IGNORE);
        info.targetRotateEncoder = encoderValue;

        shooter.setShootInfo(info);
    }

    @Override
    public boolean isFinished() {
        return shooter.onAngle;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            shooter.stop();
        }
    }
    
}
