// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.Cancel;
//import frc.robot.commands.Cancel;
import frc.robot.commands.ChangeAngleCommand;
//import frc.robot.commands.ChangeAngleCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.SetAngleCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TimeStampCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ShootInfo;
import frc.robot.subsystems.ShootPosition;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Shooter shooter = Shooter.getInstance("RobotContainer");

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

    ShootCommand amp = new ShootCommand(ShootPosition.AMP, new ShootInfo(.709, 0, .1, .25, -.7, 0.3, ShootPosition.AMP));
    m_driverController.rightTrigger().onTrue((amp));
    m_driverController.rightTrigger().onFalse(new Cancel(amp));
    // .709 subwoof shot || .525 Ferry shot
    ShootCommand shoot = new ShootCommand(ShootPosition.NORMAL, new ShootInfo(.709, 0, 1, 1, -.7, 0.6, ShootPosition.NORMAL));
    m_driverController.rightBumper().onTrue((shoot));
    m_driverController.rightBumper().onFalse(new Cancel(shoot));
    // m_driverController.povDown().onTrue(distance++);
  
    ShootCommand intake = new ShootCommand(ShootPosition.INTAKE, new ShootInfo(.51, 0, .5, .5, .2, 0.06, ShootPosition.INTAKE));
    //Command cancel = new Cancel(intake);
    m_driverController.a().onTrue(intake);//new SetAngleCommand(-0.05, 0, 0.5));
    m_driverController.a().onFalse(new Cancel(intake));//new SetAngleCommand(-0.05, 0, 0.5));
    
    //m_driverController.b().onTrue(new SetAngleCommand(-0.05, 0, 0.706));
    //m_driverController.leftBumper().onTrue(new SetAngleCommand(shooter, -0.05, 0, 0.2917));
    
    m_driverController.x().whileTrue(new ChangeAngleCommand(shooter, -0.2));
    m_driverController.y().whileTrue(new ChangeAngleCommand(shooter, 0.2));

    m_driverController.leftTrigger().whileTrue(new IntakeCommand(shooter, 0.3));
    m_driverController.leftBumper().whileTrue(new IntakeCommand(shooter, -0.3));

    //m_driverController.b().whileTrue(new TimeStampCommand(shooter));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null; //Autos.exampleAuto(shooter);
  }
}
