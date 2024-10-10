// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.RobotConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.FireCommand;
import frc.robot.commands.IntakeNoteCommand;
import frc.robot.commands.PickRobotMode;
import frc.robot.commands.ProcNoteCommand;
import frc.robot.commands.ResetClimbCommand;
import frc.robot.commands.ReverseIntakeCommand;
import frc.robot.commands.RunRobotMode;
import frc.robot.commands.SpinUpLauncher;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.SetLauncherAngleCommand;
import frc.robot.commands.SetLauncherSpeedCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.LauncherMode;
import frc.robot.subsystems.LaunchPosition;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ArmRotater;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  SendableChooser<String> autoChooser = new SendableChooser<String>();

  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 2 * Math.PI; // 3/4 of a rotation per second max angular velocity

  double upperAmpVelocity = 628.4;
  double lowerAmpVelocity = 1616;

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
  private final ClimbSubsystem climb = ClimbSubsystem.getInstance();
  private final ArmRotater arm = ArmRotater.getInstance("RobotContainer");

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final CommandXboxController m_driver2 = new CommandXboxController(1);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // Configure the trigger bindings
    configureBindings();
    registerCommands();

    autoChooser.addOption("3 Note Auto", "ThreeNoteAuto");
    autoChooser.addOption("Preload Only", "PRELOAD");
    autoChooser.addOption("2 Note Amp Side", "AmpTwoNote");
    autoChooser.addOption("2 Note Source Side", "SourceTwoNote");
    autoChooser.addOption("Preload Exit Source", "SourceExit");
    autoChooser.addOption("Amp Center Note", "AmpCenterNote2");
    autoChooser.addOption("Amp Three Note", "AmpThreeNote");
    autoChooser.addOption("Source Center Note", "SourceCenterNote");
    autoChooser.addOption("FourNoteAuto", "FourNoteAuto");

    autoChooser.setDefaultOption("3 Note Auto", "ThreeNoteAuto");

    autoChooser.addOption("Preload testing", "PreloadTesting");


    SmartDashboard.putData("Auto Chooser", autoChooser);
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
    m_driver2.a().whileTrue(drivetrain.applyRequest(() -> brake)); // to driver 2
    // joystick.b().whileTrue(drivetrain
    //     .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    
    // Launcher Controls
    m_driverController.leftTrigger().onTrue(new AutoIntakeCommand()); // Intake 628.4 1616
    m_driverController.rightBumper().onTrue(new SetLauncherAngleCommand(0.197).andThen(new WaitCommand(1)).andThen(new RunRobotMode())); // Subwoofer shot
    //m_driverController.rightTrigger().onTrue(new SetLauncherSpeedCommand(648.4, 1626).andThen(new FireCommand(5))); // Amp shot
    m_driverController.rightTrigger().onTrue(new RunRobotMode());



    // ------ TESTING SEPERATE REV MOTORS AND SHOOT CMD ------- // (done by Ryan who is not confident in coding skills, have Kevin CHECK (pun intended) it before deploying.)

    // m_driverController.rightBumper().onTrue(new SetLauncherSpeedCommand(6000, 6000)); //rev up motors
    // m_driverController.rightTrigger().onTrue(new FireCommand(true)); // shoot subwoof
    
    // -------- End of TESTING Area ^^^^ ------- //

    

    m_driver2.y().onTrue(new SetLauncherSpeedCommand(668.4, 1696).andThen(new FireCommand(5)));

    //m_driverController.rightTrigger().onTrue(new InstantCommand(() -> launcher.setPowers(0.13, 0.28)).andThen(new FireCommand(5))); // Amp shot

    m_driverController.leftBumper().onTrue(new SetLauncherAngleCommand(LaunchPosition.SUBWOOFER).andThen(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()))); // Storage position
    //m_driverController.povLeft().onTrue(new SetLauncherAngleCommand(LaunchPosition.FERRY)); // Ferry position
    // m_driver2.b().onTrue(new InstantCommand(() -> arm.setVoltage(0.05, 0.05)));
    // m_driver2.b().onFalse(new InstantCommand(() -> arm.setVoltage(0, 0)));
    //m_driver2.x().onTrue(new InstantCommand(() -> arm.setVoltage(-0.05, -0.05)));
    //m_driver2.x().onFalse(new InstantCommand(() -> arm.setVoltage(0, 0)));
    
    m_driverController.x().onTrue(new SetLauncherSpeedCommand(0, 0, 0).andThen(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()))); // Cancel all commands
    m_driverController.povDown().whileTrue(new ReverseIntakeCommand(1).alongWith(new SetLauncherSpeedCommand(-6000, -6000))).onFalse(new SetLauncherSpeedCommand(0, 0));
    
    // m_driver2.leftBumper().onTrue(new InstantCommand(() -> climb.setLeftVoltage(1)));
    // m_driver2.leftBumper().onFalse(new InstantCommand(() -> climb.setLeftVoltage(0)));
    // m_driver2.rightBumper().onTrue(new InstantCommand(() -> climb.setRightVoltage(1)));
    // m_driver2.rightBumper().onFalse(new InstantCommand(() -> climb.setRightVoltage(0)));

    // m_driver2.leftTrigger().onTrue(new InstantCommand(() -> climb.setLeftVoltage(-1)));
    // m_driver2.leftTrigger().onFalse(new InstantCommand(() -> climb.setLeftVoltage(0)));
    // m_driver2.rightTrigger().onTrue(new InstantCommand(() -> climb.setRightVoltage(-1)));
    // m_driver2.rightTrigger().onFalse(new InstantCommand(() -> climb.setRightVoltage(0)));

    m_driver2.povUp().onTrue(new SetLauncherAngleCommand(0.17).andThen(new ClimbCommand(68))); //68 is original top limit switch, 60 is for amp height
    m_driver2.povDown().onTrue(new SetLauncherAngleCommand(0.17).andThen(new ClimbCommand(0)));
    m_driver2.x().onTrue(new PickRobotMode(LauncherMode.AMP));
    m_driver2.b().onTrue(new PickRobotMode(LauncherMode.SPEAKER));
    m_driver2.leftBumper().onTrue(new ResetClimbCommand());
    //m_driver2.leftTrigger().onTrue(new SpinUpLauncher());

    //m_driver2.rightTrigger().onTrue(new SetLauncherSpeedCommand(-1000, -1000, 0.5).andThen(new ClimbCommand(10).andThen(new IntakeNoteCommand().andThen(new ProcNoteCommand()))));

  }

  public static void registerCommands() {
    NamedCommands.registerCommand("SpinUpLauncher", new SetLauncherSpeedCommand(RobotConstants.upperVelocitySpeaker, RobotConstants.lowerVelocitySpeaker));
    NamedCommands.registerCommand("IntakeCenter", new AutoIntakeCommand(0.043)); // 0.195, 0.043
    NamedCommands.registerCommand("Intake", new AutoIntakeCommand()); // 0.195, 0.043
    NamedCommands.registerCommand("IntakeOnly", new IntakeNoteCommand().alongWith(new SetLauncherAngleCommand(LaunchPosition.INTAKE)));
    NamedCommands.registerCommand("ProcessNote", new ProcNoteCommand());
    NamedCommands.registerCommand("ArmToSubwoofer", new SetLauncherAngleCommand(LaunchPosition.SUBWOOFER));
    NamedCommands.registerCommand("FireNote", new FireCommand(true));
    NamedCommands.registerCommand("SpinUpLauncherSlow", new SetLauncherSpeedCommand(5500, 5500));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    String selection = autoChooser.getSelected();
    if (selection.equals("PRELOAD")) {
      return new SetLauncherSpeedCommand(6000, 6000).andThen(new FireCommand(true));
    }
    return drivetrain.getAutoPath(autoChooser.getSelected()); //Autos.exampleAuto(shooter);
  }
}
