package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Tester extends SubsystemBase{
    private Shooter shooter = Shooter.getInstance("Tester");
    private final int numberOfLimiters = 4;
    private Limiter[] limiters = new Limiter[numberOfLimiters];
    private int selected;
    
    private double defaultValue = .4;
    static private Tester tester = null;

    enum Motors {
        INTAKE, SHOOTER_UPPER, SHOOTER_LOWER, ROTATE_SPEED
    }

    public static Tester getInstance(String name) {
        if(tester == null) {
            tester = new Tester();
        }
        System.out.println(name + " acquired Tester");
        return tester;
    }
    
    private Tester(){
        limiters[0] = new Limiter(defaultValue,"intake");
        limiters[1] = new Limiter(defaultValue,"shooterLower");
        limiters[2] = new Limiter(defaultValue,"shooterUpper");
        limiters[3] = new Limiter(defaultValue,"rotateSpeed");
    }

    public void resetAllLimiters(){
        for (int i=0; i<limiters.length; i++){
            limiters[i].value = defaultValue;
        }
    }

    public void selectNextLimiter(){
        selected++;
        selected = selected % numberOfLimiters;
    }

    public void dumpLimiters(){
        System.out.println("\n******** Limiters *******");
        for (int i=0; i<limiters.length; i++){
            System.out.print("   "+limiters[i].toString());
            if (i == selected){
                System.out.println("  selected");
            }
            System.out.println();
        }
    }

    public void makeItMove(double rotate) {
        //shooter.setShootInfo(new ShootInfo(rotate, limiters[Motors.valueOf(getName())3].value,limiters[2].value,limiters[1].value,limiters[0].value));
    }
}

class Limiter{
    double value;
    String name;
    public Limiter(double value, String name){
        this.value = value;
        this.name = name;
    }

    public String toString(){
        return name+" "+value;
    }
}