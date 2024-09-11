// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.FireCommand;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.commands.SetLauncherAngleCommand;
import frc.robot.commands.SetLauncherSpeedCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.LaunchPosition;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ArmRotater;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 2 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  // The robot's subsystems and commands are defined here...
  //private final Shooter shooter = Shooter.getInstance("RobotContainer");
  private final Launcher launcher = Launcher.getInstance("RobotContainer");
  private final Intake intake = Intake.getInstance("RobotContainer");
  private final ArmRotater arm = ArmRotater.getInstance("RobotContainer");

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // Configure the trigger bindings
    configureBindings();
    registerCommands();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {     
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //double distance = SmartDashboard.getNumber("Distance:", 0);
    // m_driverController.

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-Math.signum(joystick.getLeftY()) * Math.pow(joystick.getLeftY(), 2) * (MaxSpeed)) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-Math.signum(joystick.getLeftX()) * Math.pow(joystick.getLeftX(), 2) * (MaxSpeed)) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * (MaxAngularRate)) // Drive counterclockwise with negative X (left)
        ));
    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    
    // Launcher Controls
    m_driverController.leftTrigger().onTrue(new AutoIntakeCommand()); // Intake
    m_driverController.rightTrigger().onTrue(new SetLauncherSpeedCommand(6000, 6000).andThen(new FireCommand(true))); // Subwoofer shot
    m_driverController.povRight().onTrue(new SetLauncherSpeedCommand(648.4, 1656).andThen(new FireCommand())); // Amp shot
    m_driverController.leftBumper().onTrue(new SetLauncherAngleCommand(LaunchPosition.SUBWOOFER).andThen(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()))); // Storage position
    m_driverController.povLeft().onTrue(new SetLauncherAngleCommand(LaunchPosition.FERRY)); // Ferry position
    m_driverController.povUp().onTrue(new InstantCommand(() -> arm.changeSetpointBy(0.005)));
    m_driverController.povDown().onTrue(new InstantCommand(() -> arm.changeSetpointBy(-0.005)));
    
    m_driverController.x().onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll())); // Cancel all commands
    
    

  }

  public static void registerCommands() {
    NamedCommands.registerCommand("SpinUpLauncher", new SetLauncherSpeedCommand(6000, 6000));
    NamedCommands.registerCommand("Intake", new AutoIntakeCommand(0.043)); // 0.195, 0.043
    NamedCommands.registerCommand("FireNote", new FireCommand(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivetrain.getAutoPath("Test2"); //Autos.exampleAuto(shooter);
  }
}
