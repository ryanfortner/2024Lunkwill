// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Load Note Command
 * Run indexer motor for a specified amount of seconds
 * and at a specified speed.
 */

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class LoadNoteCommand extends Command {
  private final Timer timer = new Timer();

  private final Shooter shooter;

  private final double speed;

  /** Creates a new LoadNoteCommand. */
  public LoadNoteCommand(Shooter shooter, double speed) {
    this.shooter = shooter;

    this.speed = speed;

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("LOAD NOTE");
    shooter.setDriveMotorSpeed(-speed);
    shooter.setIndexerMotorSpeed(-speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
