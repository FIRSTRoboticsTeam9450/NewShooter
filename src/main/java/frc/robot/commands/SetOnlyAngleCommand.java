package frc.robot.commands;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Dictionary;
import java.util.Scanner;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.InfoParams;
import frc.robot.subsystems.ShootInfo;
import frc.robot.subsystems.ShootPosition;

import frc.robot.subsystems.Shooter;

public class SetOnlyAngleCommand extends Command {

    Shooter shooter = Shooter.getInstance("SetOnlyAngleCommand");
    ShootInfo info;
    ShootPosition shotType;
    boolean finished;

    public SetOnlyAngleCommand(ShootPosition type) {
        this.shotType = type;
    }

    @Override
    public void initialize() {
        finished = false;
        prepareShot();
    }

    @Override
    public void execute() {
        if (shooter.onAngle) {
            finished = true;
        }
    }

    //prepare the shooter motors for the shot
    public void prepareShot() {
        info = new ShootInfo(InfoParams.IGNORE);
        info.lowerShooterPower = 0.0;
        info.upperShooterPower = 0.0;

        info.targetRotateEncoder = convertToEncoder(shotType);
        shooter.setShootInfo(info);
    }

    public double convertToEncoder(ShootPosition shotType) {
        double targetRotateEncoder = 0.0;

        switch (shotType) {
            case AMP:
                targetRotateEncoder = 0.0002;
                break;
                // 3/8 inch = .01 encoder  
            case SUBWOOFER: // mechanical max: left: .71/.2108 right:.705/.2108 || mechanical min: left .499/.000 right: .494/-.000
                targetRotateEncoder = 0.2;
                break;
            
            case INTAKE:
                targetRotateEncoder = 0.0002;
                break;
                
            case HORIZONTAL:
                targetRotateEncoder = 0.14;
            default:
        }
        return targetRotateEncoder;
    }
    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }

}
