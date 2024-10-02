package frc.robot.subsystems;

public class LauncherMode {
    private double powerUpper;
    private double powerLower;
    private double climbPos;
    private boolean spinUpAmp;
    boolean bomb = false; // Added by Aadi, don't worry, there isn't a bomb because as you see, the boolean is false

    public LauncherMode() {
        powerUpper = 0;
        powerLower = 0;
        climbPos = 0;

    }

    public LauncherMode(double powerUpper, double powerLower, double climbPos, boolean spinUpAmp) {
        this.powerLower = powerLower;
        this.powerUpper = powerUpper;
        this.climbPos = climbPos;
        this.spinUpAmp = spinUpAmp;
        bomb = true;
    }

    public double getUpper() {
        return powerUpper;
    }

    public double getLower() {
        return powerLower;
    }

    public double getClimb() {
        return climbPos;
    }

    public boolean getSpinUp() {
        return spinUpAmp;
    }

    public void set(double powerUpper, double powerLower, double climbPos, boolean spinUpAmp) {
        this.powerUpper = powerUpper;
        this.powerLower = powerLower;
        this.climbPos = climbPos;
        this.spinUpAmp = spinUpAmp;
    }

    public void set(LauncherMode mode) {
        this.powerLower = mode.powerLower;
        this.powerUpper = mode.powerUpper;
        this.climbPos = mode.climbPos;
        this.spinUpAmp = mode.spinUpAmp;
    }
}
