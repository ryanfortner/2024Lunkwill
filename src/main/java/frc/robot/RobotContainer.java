// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.LoadNoteCommand;
import frc.robot.commands.ShootNoteCommand;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Create swerve subsystem
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  
  private static final Shooter shooter = new Shooter(ShooterConstants.indexerMotorId, ShooterConstants.driveMotorId);

  /*
   * Create Joystick object
   * Must be Joystick and not CommandJoystick,
   * which lacks getRawButton functionality
   */
  private final static Joystick joystick =
      new Joystick(OperatorConstants.JoystickPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Send axes & buttons from joystick to SwerveJoystickCommand,
      // which will govern the SwerveSubsystem
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCommand(
      swerveSubsystem, 
      () -> joystick.getRawAxis(OperatorConstants.JoystickTranslationAxis),
      () -> joystick.getRawAxis(OperatorConstants.JoystickStrafeAxis), 
      () -> joystick.getRawAxis(OperatorConstants.JoystickRotationAxis),
      () -> joystick.getRawAxis(OperatorConstants.JoystickSliderAxis),
      () -> joystick.getRawButton(OperatorConstants.JoystickRobotRelative)
    ));

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

    /* LOAD NOTE */
    /* for 5 sec @0.5x speed */
    new JoystickButton(joystick, OperatorConstants.JoystickLoadNote)
      .onTrue(new LoadNoteCommand(shooter, 0.5, 5));
    
    /* SHOOT NOTE */
    /* for 5 sec, with indexer @0.5x speed */
    new JoystickButton(joystick, OperatorConstants.JoystickShootNote)
      .onTrue(new ShootNoteCommand(shooter, 0.5, 5));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}
