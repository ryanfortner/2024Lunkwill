// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private final VictorSPX indexerMotor;
  private final VictorSPX driveMotor;

  /** Creates a new Shooter. */
  public Shooter(int indexerMotorId, int driveMotorId) {
    indexerMotor = new VictorSPX(indexerMotorId);
    driveMotor = new VictorSPX(driveMotorId);
  }

  public void setBothMotorsSpeed(double speed) {
    driveMotor.set(VictorSPXControlMode.PercentOutput, speed);
    indexerMotor.set(VictorSPXControlMode.PercentOutput, speed);

  }

  public void setDriveMotorSpeed(double speed) {
    driveMotor.set(VictorSPXControlMode.PercentOutput, speed);
  }

  public void setIndexerMotorSpeed(double speed) {
    indexerMotor.set(VictorSPXControlMode.PercentOutput, speed);
  }

  public void stopIndexMotor() {
    indexerMotor.set(VictorSPXControlMode.PercentOutput, 0);
  }

  public void stopDriveMotor() {
    driveMotor.set(VictorSPXControlMode.PercentOutput, 0);
  }

  public void stopAllMotors() {
    indexerMotor.set(VictorSPXControlMode.PercentOutput, 0);
    driveMotor.set(VictorSPXControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {

  }
}
