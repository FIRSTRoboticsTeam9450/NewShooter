package frc.robot.commands;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Dictionary;
import java.util.Scanner;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShootInfo;
import frc.robot.subsystems.ShootPosition;

import frc.robot.subsystems.Shooter;

public class ShootCommand extends Command {

    Shooter shooter = Shooter.getInstance("SetAngleCommand");
    double power;
    double encoderValue;
    boolean auto;
    double angle;
    ShootInfo info;
    ShootPosition shotType;
    boolean everFalse = false;
    boolean runOnceShoot = true;
    public ShootCommand(ShootPosition type) {
        this.shotType = type;
    }

    @Override
    public void initialize() {
        System.out.println("ShootCommand ");
        prepareShot();
    }

    @Override
    public void execute() {
        switch (shotType) {
            case INTAKE:
                if (shooter.onAngle && runOnceShoot) {
                    info = new ShootInfo(Double.MAX_VALUE);
                    info.intakeSpeed = 0.3;

                    shooter.setShootInfo(info);

                    this.runOnceShoot = false;
                }
            default:
                 if (shooter.shooterMotorsOn && shooter.onAngle && runOnceShoot) {
                    info = new ShootInfo(Double.MAX_VALUE);
                    info.intakeSpeed = 0.3;
      
                    shooter.setShootInfo(info);
      
                    this.runOnceShoot = false;
                }
        }

       
    }

    //prepare the shooter motors for the shot
    public void prepareShot() {
        info = new ShootInfo(Double.MAX_VALUE);

        //shot type:
        switch (shotType) {
        case AMP:
            info.lowerShooterPower = 0.25;
            info.upperShooterPower = 0.1;
            info.targetRotateEncoder = 0.1;
            break;

        case SUBWOOFER:
            info.lowerShooterPower = 1;
            info.upperShooterPower = 1;
            info.targetRotateEncoder = 0.708 - 0.497;
            break;

        case INTAKE:
            info.lowerShooterPower = -0.1;
            info.upperShooterPower = -0.1;
            info.targetRotateEncoder = 0.0002;
        default:
        }
        

        shooter.setShootInfo(info);
    }

    @Override
    public void end(boolean interrupted) {
        info.intakeSpeed = -0.3;

        shooter.stop();

    }
    
    @Override
    public boolean isFinished() {
        switch (shotType) {
        case INTAKE:
            
            return shooter.noteIn;
        
        default:
            return !shooter.noteIn;
        }
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
    double[] encoderValues = {0.2917
        ,0.2923843333
        ,0.2930686667
        ,0.293753
        ,0.2944373333
        ,0.2951216667
        ,0.295806
        ,0.2964903333
        ,0.2971746667
        ,0.297859
        ,0.2985433333
        ,0.2992276667
        ,0.299912
        ,0.3005963333
        ,0.3012806667
        ,0.301965
        ,0.3026493333
        ,0.3033336667
        ,0.304018
        ,0.3047023333
        ,0.3053866667
        ,0.306071
        ,0.3067553333
        ,0.3074396667
        ,0.308124
        ,0.3088083333
        ,0.3094926667
        ,0.310177
        ,0.3108613333
        ,0.3115456667
        ,0.31223
        ,0.3129143333
        ,0.3135986667
        ,0.314283
        ,0.3149673333
        ,0.3156516667
        ,0.316336
        ,0.3170203333
        ,0.3177046667
        ,0.318389
        ,0.3190733333
        ,0.3197576667
        ,0.320442
        ,0.3211263333
        ,0.3218106667
        ,0.322495
        ,0.3231793333
        ,0.3238636667
        ,0.324548
        ,0.3252323333
        ,0.3259166667
        ,0.326601
        ,0.3272853333
        ,0.3279696667
        ,0.328654
        ,0.3293383333
        ,0.3300226667
        ,0.330707
        ,0.3313913333
        ,0.3320756667
        ,0.33276
        ,0.3334443333
        ,0.3341286667
        ,0.334813
        ,0.3354973333
        ,0.3361816667
        ,0.336866
        ,0.3375503333
        ,0.3382346667
        ,0.338919
        ,0.3396033333
        ,0.3402876667
        ,0.340972
        ,0.3416563333
        ,0.3423406667
        ,0.343025
        ,0.3437093333
        ,0.3443936667
        ,0.345078
        ,0.3457623333
        ,0.3464466667
        ,0.347131
        ,0.3478153333
        ,0.3484996667
        ,0.349184
        ,0.3498683333
        ,0.3505526667
        ,0.351237
        ,0.3519213333
        ,0.3526056667
        ,0.35329
        ,0.3539743333
        ,0.3546586667
        ,0.355343
        ,0.3560273333
        ,0.3567116667
        ,0.357396
        ,0.3580803333
        ,0.3587646667
        ,0.359449
        ,0.3601333333
        ,0.3608176667
        ,0.361502
        ,0.3621863333
        ,0.3628706667
        ,0.363555
        ,0.3642393333
        ,0.3649236667
        ,0.365608
        ,0.3662923333
        ,0.3669766667
        ,0.367661
        ,0.3683453333
        ,0.3690296667
        ,0.369714
        ,0.3703983333
        ,0.3710826667
        ,0.371767
        ,0.3724513333
        ,0.3731356667};
}
