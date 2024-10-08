package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Controls the intake motor and dual LaserCAN sensors */
public class Intake extends SubsystemBase {

    CANSparkFlex motorIntake = new CANSparkFlex(32, MotorType.kBrushless);
    private static Intake intake;

    // LaserCANs are configured with a medianfilter, which means the last 3 reults are averaged together
    // this smooths out the output nicely
    private LaserCan entryLaser;
    private LaserCan.Measurement entryMeasurement; 
    MedianFilter entryMedianDistance = new MedianFilter(3);
    double entryLaserDistance;

    private LaserCan exitLaser;
    private LaserCan.Measurement exitMeasurement; 
    MedianFilter exitMedianDistance = new MedianFilter(3);
    double exitLaserDistance;

    //** Creates a new Intake object */
    public Intake() {
        motorIntake.restoreFactoryDefaults();
        motorIntake.setInverted(true);

        // Configure entry laser sensor
        entryLaser = new LaserCan(40);
        try {
        entryLaser.setRangingMode(LaserCan.RangingMode.LONG);
        entryLaser.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 4, 4));
        entryLaser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch(ConfigurationFailedException e) {
        System.out.println("Entry laser configuration failed " + e);
        }
        entryMeasurement = entryLaser.getMeasurement();

        // Configure exit laser sensor
        exitLaser = new LaserCan(41);
        try {
        exitLaser.setRangingMode(LaserCan.RangingMode.LONG);
        exitLaser.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 4, 4));
        exitLaser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch(ConfigurationFailedException e) {
        System.out.println("Entry laser configuration failed " + e);
        }
        exitMeasurement = exitLaser.getMeasurement();
    }

    /**
     * Returns an instance of the Intake class - use this instead of passing to commands
     * @param name The name of the class requesting the Intake instance
     * @return An instance of Intake
     */
    public static Intake getInstance(String name) {
        System.out.println(name + " has acquired Intake");
        if (intake == null) {
            intake = new Intake();
        }
        return intake;
    }

    /** Sets the intake motor power */
    public void setPower(double power) {
        motorIntake.set(power);
    }

    /** Updates the measurements from the lasers */
    public void updateLasers() {
        try {
          entryMeasurement = entryLaser.getMeasurement();
          if(entryMeasurement != null && entryMeasurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            entryLaserDistance = entryMedianDistance.calculate(entryMeasurement.distance_mm);
            Logger.recordOutput("Intake/EntryLaser", entryLaserDistance);
          }
        } catch (Exception e) {
          e.printStackTrace();
        }

        try {
          exitMeasurement = exitLaser.getMeasurement();
          if(exitMeasurement != null && exitMeasurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            exitLaserDistance = exitMedianDistance.calculate(exitMeasurement.distance_mm);
            Logger.recordOutput("Intake/ExitLaser", exitLaserDistance);
          }
        } catch (Exception e) {
          e.printStackTrace();
        }
    }

    /**
     * Returns the distance read by the entry laser
     * @return distance (millimeters)
     */
    public double getEntryLaserDistance() {
        return entryLaserDistance;
    }

    /**
     * Returns the distance read by the exit laser
     * @return distance (millimeters)
     */
    public double getExitLaserDistance() {
        return exitLaserDistance;
    }

    @Override
    public void periodic() {
        updateLasers();
        Logger.recordOutput("Intake/IntakeVelocity", motorIntake.getEncoder().getVelocity());
        Logger.recordOutput("Intake/IntakePower", motorIntake.get());
        Logger.recordOutput("Intake/HasNote", entryLaserDistance < 20);
        
    }
  
}
