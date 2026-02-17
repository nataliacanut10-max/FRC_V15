// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;

public class TankDriveCommand extends Command {
  private final CANDriveSubsystem CANDriveSubsystem;
  private DoubleSupplier x, y;
  private double speed;

  public TankDriveCommand(CANDriveSubsystem CANDriveSubsystem, DoubleSupplier x, DoubleSupplier y, double speed) {
    this.CANDriveSubsystem = CANDriveSubsystem;
    this.x = x;
    this.y = y;
    this.speed = speed;
    addRequirements(CANDriveSubsystem);
  }

  @Override
  public void execute() {
    double leftSide = (x.getAsDouble() + y.getAsDouble()) * speed;
    double rightSide = (x.getAsDouble() - y.getAsDouble()) * speed;

    CANDriveSubsystem.drive(leftSide, rightSide);
  }

  @Override
  public void end(boolean interrupted) {
    CANDriveSubsystem.drive(0.0, 0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
