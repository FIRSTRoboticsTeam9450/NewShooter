// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.Cancel;
import frc.robot.commands.ChangeAngleCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.FireNote;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.SetAngleCommand;
import frc.robot.commands.SetLauncherAngle;
import frc.robot.commands.SetOnlyAngleCommand;
//import frc.robot.commands.SetAngleCommandTest;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ShootNowCommand;
import frc.robot.commands.TimeStampCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.InfoParams;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.ShootInfo;
import frc.robot.subsystems.ShootPosition;
import frc.robot.subsystems.Shooter;

import java.util.Set;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.TunerConstants;
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

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // Configure the trigger bindings
    configureBindings();
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

    
    //SetAngleCommand auto = new SetAngleCommand(ShootPosition.AUTO);
    //m_driverController.leftBumper().onTrue(auto);
    //m_driverController.povDown().onFalse(new Cancel(auto));
    

    // OLD COMMANDS

    m_driverController.leftBumper().onTrue(new InstantCommand(() -> launcher.setVelocities(6000, 0.1)));
    m_driverController.leftBumper().onFalse(new InstantCommand(() -> launcher.setVelocities(0, 0)));

    /*
    m_driverController.povRight().onTrue(new ShootNowCommand(ShootPosition.AMP));
    
    IntakeCommand moveNoteForward = new IntakeCommand(shooter,.1);
    m_driverController.povUp().onTrue(moveNoteForward);
    m_driverController.povUp().onFalse(new Cancel(moveNoteForward));

    IntakeCommand moveNoteBack = new IntakeCommand(shooter,-.1);
    m_driverController.povDown().onTrue(moveNoteBack);
    m_driverController.povDown().onFalse(new Cancel(moveNoteBack));

    m_driverController.rightTrigger().onTrue(new ShootNowCommand(ShootPosition.SUBWOOFER));//(new ShootCommand(ShootPosition.SUBWOOFER)));
    m_driverController.leftTrigger().onTrue((new ShootCommand(ShootPosition.INTAKE).andThen(new SetLauncherAngle(0.2))));
    m_driverController.rightBumper().onTrue(new ShootCommand(ShootPosition.HORIZONTAL));
    m_driverController.leftBumper().onTrue(new SetOnlyAngleCommand(ShootPosition.SUBWOOFER));
    */
    m_driverController.x().onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));
    
    // SetAngleCommandTest up = new SetAngleCommandTest(new ShootInfo(.212, 0.0, 0.0, 0.0, 0.0));
    // m_driverController.povRight().onTrue(up);
    // m_driverController.povRight().onFalse(new Cancel(up));

    // SetAngleCommandTest down = new SetAngleCommandTest(new ShootInfo(0, 0, 0, 0, 0));
    // m_driverController.povDown().onTrue(down);
    // m_driverController.povDown().onFalse(new Cancel(down));
    
    // SetAngleCommandTest ferry = new SetAngleCommandTest(new ShootInfo(.028, 0, 0, 0, 0));
    // m_driverController.povLeft().onTrue(ferry);
    // m_driverController.povLeft().onFalse(new Cancel(ferry));
    
    // SetAngleCommandTest straight = new SetAngleCommandTest(new ShootInfo(.1, 0, 0, 0, 0));
    // m_driverController.povUp().onTrue(straight);
    // m_driverController.povUp().onFalse(new Cancel(straight));
    
    
    
    //m_driverController.x().whileTrue(new ChangeAngleCommand(shooter, -0.2));
    //m_driverController.y().whileTrue(new ChangeAngleCommand(shooter, 0.2));

    


    //m_driverController.leftTrigger().whileTrue(new IntakeCommand(shooter, 0.3));
    //m_driverController.leftBumper().whileTrue(new IntakeCommand(shooter, -0.3));

    //m_driverController.b().whileTrue(new TimeStampCommand(shooter));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivetrain.getAutoPath("Test"); //Autos.exampleAuto(shooter);
  }
}
