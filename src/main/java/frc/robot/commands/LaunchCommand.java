package frc.robot.commands;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Dictionary;
import java.util.Scanner;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.InfoParams;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.ShootInfo;
import frc.robot.subsystems.ShootPosition;

import frc.robot.subsystems.Shooter;

public class LaunchCommand extends Command {

    Intake intake = Intake.getInstance("LaunchCommand");
    Carriage carriage = Carriage.getInstance("LaunchCommand");
    Launcher launcher = Launcher.getInstance("LaunchCommand");
    double power;
    double encoderValue;
    boolean auto;
    double angle;
    ShootInfo info;
    ShootPosition shotType;
    boolean everFalse = false;
    boolean runOnceShoot = true;
    SetLauncherAngle horizontalCommand = new SetLauncherAngle(encoderValue);
    Timer timer;

    public LaunchCommand(ShootPosition type) {
        this.shotType = type;
    }

    @Override
    public void initialize() {
        System.out.println("LaunchCommand ");
        prepareShot();
    }

    @Override
    public void execute() {
        if (launcher.atSpeed() && carriage.onAngle && runOnceShoot) {
            intake.setPower(1);

            this.runOnceShoot = false;
            timer.reset();
            timer.restart();
        }
       
    }
    //prepare the shooter motors for the shot
    public void prepareShot() {
        runOnceShoot = true;
        double targetRotateEncoderAuto = getEncoderValue(133);
        System.out.println(targetRotateEncoderAuto);
        SmartDashboard.putNumber("Auto generated target encoder", targetRotateEncoderAuto);
        //shot type:
        switch (shotType) {
        case AMP:
            launcher.setPowers(0.25, 0.1);
            carriage.setTarget(0.0002);
            break;
            // 3/8 inch = .01 encoder  
        case SUBWOOFER: // mechanical max: left: .71/.2108 right:.705/.2108 || mechanical min: left .499/.000 right: .494/-.000
            launcher.setPowers(1, 1);
            carriage.setTarget(0.21);
            break;
        
        case INTAKE:
            launcher.setPowers(-0.1, -0.1);
            carriage.setTarget(0.0002);
            break;
            
        case HORIZONTAL:
            launcher.setPowers(1, 1);
            carriage.setTarget(0.14);
        default:
        }
        
    }

    public double getEncoderValue(double distance) {
        
        angle = Math.asin((80/Math.sqrt(Math.pow(distance+3, 2) + Math.pow(80, 2))));
        angle *= (180/3.14);
        System.out.println("ANGLE:::" + angle);
        double targetRotateEncoder = 0;
        int mid = angleValues.length/2;
        int left = 0;
        int right = angleValues.length-1;
        while(right - left != 1) {

            if(angleValues[mid] > angle) {
                left = mid;
            }
            else {
                right = mid;
            }
            mid = left + ((right - left) / 2);
        }
        targetRotateEncoder = encoderValues[left] - .497; // eventually change to constant
        System.out.println("ENCODER VALUE: " + targetRotateEncoder);
        return targetRotateEncoder;
    }

    @Override
    public void end(boolean interrupted) {
        launcher.setPowers(0, 0);
        intake.setPower(0);
    }
    
    @Override
    public boolean isFinished() {
        return !runOnceShoot && timer.hasElapsed(1);
    }

    double[] angleValues = {50.59359786
      ,47.70163251
      ,46.14099054
      ,44.98544167
      ,43.9902074
      ,43.15865574
      ,42.43388811
      ,41.75527765
      ,41.15183708
      ,40.56902586
      ,40.05352506
      ,39.56336111
      ,39.09192643
      ,38.65650414
      ,38.23561229
      ,37.8401917
      ,37.46342451
      ,37.09613585
      ,36.74724062
      ,36.42056284
      ,36.10565647
      ,35.79239493
      ,35.50360444
      ,35.20511374
      ,34.9231826
      ,34.65655162
      ,34.4055305
      ,34.15525414
      ,33.91254556
      ,33.67228481
      ,33.44725727
      ,33.22096059
      ,33.01016543
      ,32.79901459
      ,32.59471956
      ,32.38570599
      ,32.19693031
      ,32.00407863
      ,31.81480674
      ,31.63744221
      ,31.46293402
      ,31.29471502
      ,31.12310751
      ,30.95454027
      ,30.79553392
      ,30.64156278
      ,30.49293888
      ,30.34418253
      ,30.20237973
      ,30.050031
      ,29.90919786
      ,29.7745695
      ,29.64666006
      ,29.52387699
      ,29.40048123
      ,29.271499
      ,29.14433203
      ,29.03184861
      ,28.91290387
      ,28.80397575
      ,28.68972398
      ,28.58829578
      ,28.48368971
      ,28.37867028
      ,28.27310338
      ,28.17862659
      ,28.07852813
      ,27.9898726
      ,27.89254942
      ,27.80955837
      ,27.71963079
      ,27.63791281
      ,27.55580532
      ,27.47332894
      ,27.38816478
      ,27.31416462
      ,27.24104556
      ,27.16429145
      ,27.08673993
      ,27.02675453
      ,26.94949632
      ,26.8870213
      ,26.82072456
      ,26.76346983
      ,26.69577744
      ,26.63956831
      ,26.58427002
      ,26.52646568
      ,26.47849615
      ,26.43154103
      ,26.37503044
      ,26.32048538
      ,26.2843977
      ,26.23798077
      ,26.18632043
      ,26.14410145
      ,26.11166028
      ,26.06758145
      ,26.03719872
      ,25.99046345
      ,25.9621819
      ,25.92841768
      ,25.89852514
      ,25.86532615
      ,25.83559564
      ,25.81097983
      ,25.78386771
      ,25.770402
      ,25.73930735
      ,25.71766056
      ,25.70462659
      ,25.68778188
      ,25.67714724
      ,25.65519903
      ,25.64950239
      ,25.63111518
      ,25.62422275
      ,25.61813015
      ,25.60490085
      ,25.59816156};
   double[] encoderValues = {0.708
       ,0.7072966667
       ,0.7065933333
       ,0.70589
       ,0.7051866667
       ,0.7044833333
       ,0.70378
       ,0.7030766667
       ,0.7023733333
       ,0.70167
       ,0.7009666667
       ,0.7002633333
       ,0.69956
       ,0.6988566667
       ,0.6981533333
       ,0.69745
       ,0.6967466667
       ,0.6960433333
       ,0.69534
       ,0.6946366667
       ,0.6939333333
       ,0.69323
       ,0.6925266667
       ,0.6918233333
       ,0.69112
       ,0.6904166667
       ,0.6897133333
       ,0.68901
       ,0.6883066667
       ,0.6876033333
       ,0.6869
       ,0.6861966667
       ,0.6854933333
       ,0.68479
       ,0.6840866667
       ,0.6833833333
       ,0.68268
       ,0.6819766667
       ,0.6812733333
       ,0.68057
       ,0.6798666667
       ,0.6791633333
       ,0.67846
       ,0.6777566667
       ,0.6770533333
       ,0.67635
       ,0.6756466667
       ,0.6749433333
       ,0.67424
       ,0.6735366667
       ,0.6728333333
       ,0.67213
       ,0.6714266667
       ,0.6707233333
       ,0.67002
       ,0.6693166667
       ,0.6686133333
       ,0.66791
       ,0.6672066667
       ,0.6665033333
       ,0.6658
       ,0.6650966667
       ,0.6643933333
       ,0.66369
       ,0.6629866667
       ,0.6622833333
       ,0.66158
       ,0.6608766667
       ,0.6601733333
       ,0.65947
       ,0.6587666667
       ,0.6580633333
       ,0.65736
       ,0.6566566667
       ,0.6559533333
       ,0.65525
       ,0.6545466667
       ,0.6538433333
       ,0.65314
       ,0.6524366667
       ,0.6517333333
       ,0.65103
       ,0.6503266667
       ,0.6496233333
       ,0.64892
       ,0.6482166667
       ,0.6475133333
       ,0.64681
       ,0.6461066667
       ,0.6454033333
       ,0.6447
       ,0.6439966667
       ,0.6432933333
       ,0.64259
       ,0.6418866667
       ,0.6411833333
       ,0.64048
       ,0.6397766667
       ,0.6390733333
       ,0.63837
       ,0.6376666667
       ,0.6369633333
       ,0.63626
       ,0.6355566667
       ,0.6348533333
       ,0.63415
       ,0.6334466667
       ,0.6327433333
       ,0.63204
       ,0.6313366667
       ,0.6306333333
       ,0.62993
       ,0.6292266667
       ,0.6285233333
       ,0.62782
       ,0.6271166667
       ,0.6264133333
       ,0.62571
       ,0.6250066667
       ,0.6243033333};
}
