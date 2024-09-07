package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    CANSparkFlex motorIntake = new CANSparkFlex(32, MotorType.kBrushless);
    private static Intake intake;

    private LaserCan entryLaser;
    private LaserCan.Measurement entryMeasurement; 
    MedianFilter entryMedianDistance = new MedianFilter(3);
    double entryLaserDistance;

    private LaserCan exitLaser;
    private LaserCan.Measurement exitMeasurement; 
    MedianFilter exitMedianDistance = new MedianFilter(3);
    double exitLaserDistance;

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

    // Returns an instance of the Intake class - use this instead of passing to commands
    public static Intake getInstance(String name) {
        System.out.println(name + " has acquired Intake");
        if (intake == null) {
            intake = new Intake();
        }
        return intake;
    }

    // Sets the intake motor power
    public void setPower(double power) {
        motorIntake.set(power);
    }

    // Updates reads the measurements from the lasers
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

    // Returns the distance read by the entry laser
    public double getEntryLaserDistance() {
        return entryLaserDistance;
    }

    // Returns the distance read by the exit laser
    public double getExitLaserDistance() {
        return exitLaserDistance;
    }

    @Override
    public void periodic() {
        updateLasers();
    }




    
}
