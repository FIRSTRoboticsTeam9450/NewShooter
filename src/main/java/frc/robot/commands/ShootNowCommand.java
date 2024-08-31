package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.InfoParams;
import frc.robot.subsystems.ShootInfo;
import frc.robot.subsystems.ShootPosition;
import frc.robot.subsystems.Shooter;

public class ShootNowCommand extends Command {

    Shooter shooter = Shooter.getInstance("SetNowCommand");
    ShootPosition shootPosition;
    int delay;

    public ShootNowCommand(ShootPosition shootPosition) {
        this.shootPosition = shootPosition;
    }

    @Override
    public void initialize() {
        System.out.println("ShootNowCommand ");
        prepareShot(shootPosition);
        delay = 50;
    }

    @Override
    public void execute() {
    }

    //prepare the shooter motors for the shot
    public void prepareShot(ShootPosition shotType) {
        ShootInfo info = new ShootInfo(InfoParams.IGNORE);

        switch (shotType) {
            case AMP:
                info.lowerShooterPower = 0.25;
                info.upperShooterPower = 0.1;
                break;
                // 3/8 inch = .01 encoder  
            case SUBWOOFER: // mechanical max: left: .71/.2108 right:.705/.2108 || mechanical min: left .499/.000 right: .494/-.000
                info.lowerShooterPower = 1;
                info.upperShooterPower = 1;
                break;
            
            case INTAKE:
                info.lowerShooterPower = -0.1;
                info.upperShooterPower = -0.1;
                break;
                
            case HORIZONTAL:
                info.lowerShooterPower = 1;
                info.upperShooterPower = 1;
            default:
            }
            
        info.intakeSpeed = 1;
        
        shooter.setShootInfo(info);
    }


    @Override
    public void end(boolean interrupted) {
        shooter.stop();

    }
    
    @Override
    public boolean isFinished() {
        return (!shooter.noteIn) && (delay-- > 0);
    }
}
