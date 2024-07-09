package frc.robot.commands;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SetAngleCommand extends Command {

    Shooter shooter;
    double power;
    double distance;
    double encoderValue;
    boolean auto;
    double angle;
    public SetAngleCommand(Shooter shooter, double power, double distance, double encoderValue) {
        this.shooter = shooter;
        this.power = power;
        this.distance = distance;
        if(distance == 0) {
            auto = false;
        }
        else {
            auto = true;
        }

        this.encoderValue = encoderValue;
        
    }

    @Override
    public void initialize() {
        if(auto) {
            angle = Math.asin((80/Math.sqrt(Math.pow(distance, 2) + Math.pow(80, 2))));
            try (Scanner console = new Scanner(new File("C:\\NN\\Projects\\FRCTest\\NewShooter\\src\\main\\java\\frc\\robot\\angle data.csv"))) {
                console.nextLine();
                boolean done = false;
                while(console.hasNextLine() && !done) {
                    
                    Scanner line = new Scanner(console.nextLine());
                    line.useDelimiter(",");
                    line.next();
                    if(Math.abs(Double.parseDouble(line.next()) - angle) <= 1) {
                        done = true;
                        encoderValue = Double.parseDouble(line.next());
                    }
                    else {
                        line.next();
                    }
                }
            } catch (FileNotFoundException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
            encoderValue = angle;
        }
        shooter.setRotationSpeed(power, encoderValue);
    }

    @Override
    public void execute() {
        //System.out.println("RUNNING!!!");
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setRotationSpeed(0, 0);
        //System.out.println("END.");
    }
    
    @Override
    public boolean isFinished() {
        return shooter.rotateTil(encoderValue);
    }
}
