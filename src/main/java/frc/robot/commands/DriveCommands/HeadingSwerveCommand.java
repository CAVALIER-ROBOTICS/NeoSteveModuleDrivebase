// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class HeadingSwerveCommand extends CommandBase {
  DriveSubsystem driveSubsystem;

  DoubleSupplier leftX, leftY, rightX;

  public HeadingSwerveCommand(DriveSubsystem dSub, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rX) {
    this.driveSubsystem = dSub;
    this.leftX = x;
    this.leftY = y;
    this.rightX = rX;

    addRequirements(dSub);
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      leftX.getAsDouble(),
      leftY.getAsDouble(),
      rightX.getAsDouble(),
      driveSubsystem.getAngle());

    if(Math.abs(rightX.getAsDouble()) < .01) {
      driveSubsystem.adjustedDrive(fieldSpeeds, rightX.getAsDouble());
    } else {
      driveSubsystem.zeroGyroscope();
      driveSubsystem.drive(fieldSpeeds);
      driveSubsystem.resetDesiredHeading();
    }
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
