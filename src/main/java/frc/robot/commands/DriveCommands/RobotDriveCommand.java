// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class RobotDriveCommand extends CommandBase {
  DriveSubsystem driveSubsystem;

  DoubleSupplier leftX, leftY, rightX;

  public RobotDriveCommand(DriveSubsystem dSub, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rX) {
    this.driveSubsystem = dSub;
    this.leftX = x;
    this.leftY = y;
    this.rightX = rX;

    addRequirements(dSub);
  }

  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds fieldSpeeds = new ChassisSpeeds(
        leftX.getAsDouble(),
        leftY.getAsDouble(),
        rightX.getAsDouble());

    ChassisSpeeds speeds = fieldSpeeds;

    driveSubsystem.drive(speeds);
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
