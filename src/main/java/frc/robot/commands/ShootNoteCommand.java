// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Shoot Note Command */
/* Spin up main motor
 * Start timer
 * Run indexer
 * Stop after x seconds
 */

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShootNoteCommand extends Command {

  private final Timer timer = new Timer();

  private final Shooter shooter;

  private final double duration, indexerSpeed;

  /** Creates a new ShootNoteCommand. */
  public ShootNoteCommand(Shooter shooter, double indexerSpeed, double duration) {
    this.duration = duration;
    this.indexerSpeed = indexerSpeed;
    this.shooter = shooter;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();

    // spin up drive motor
    shooter.setDriveMotorSpeed(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Run indexer at specified speed
    shooter.setIndexerMotorSpeed(indexerSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    shooter.stopAllMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= duration;
  }
}
