// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;

//import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands.ABV2Command;
import frc.robot.commands.DriveCommands.FieldDriveCommand;
import frc.robot.commands.DriveCommands.HeadingSwerveCommand;
import frc.robot.commands.DriveCommands.LockWheelsCommand;
import frc.robot.commands.DriveCommands.RedAlliance;
import frc.robot.commands.DriveCommands.RobotDriveCommand;
//import frc.robot.commands.HeadingSwerveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NEOTestSub;

public class RobotContainer {
  public static final XboxController driver = new XboxController(0);
  public static final XboxController operator = new XboxController(1);

  DriveSubsystem driveSubsystem = new DriveSubsystem();
  NEOTestSub neoTestSub = new NEOTestSub();

  public RobotContainer() {
    driveSubsystem.setDefaultCommand(new FieldDriveCommand(
        driveSubsystem,
        () -> driver.getLeftY() * 4,
        () -> driver.getLeftX() * -4,
        () -> driver.getRightX() * 2 * Math.PI));

    configureBindings();
  }

  public boolean getLeftTrigger(XboxController c) {
    return c.getLeftTriggerAxis() > .05;
  }

  public boolean getRightTrigger(XboxController c) {
    return c.getRightTriggerAxis() > .05;
  }

  private void configureBindings() {
    JoystickButton resetGyro = new JoystickButton(driver, 4);
    JoystickButton lockWheels = new JoystickButton(driver, 3);
    JoystickButton babyMode = new JoystickButton(driver, 1);

    // command schedulers

    resetGyro.onTrue(new InstantCommand(driveSubsystem::zeroGyroscope));

    lockWheels.toggleOnTrue(new LockWheelsCommand(driveSubsystem));

    babyMode.toggleOnTrue(new FieldDriveCommand(
        driveSubsystem,
        () -> driver.getLeftX() / -10,
        () -> driver.getLeftY() / 10,
        () -> driver.getRightX() / -5));

  }

  public Command getSinglePath(String name) {
    return new SequentialCommandGroup(
        new InstantCommand(driveSubsystem::zeroGyroscope),
        // new InstantCommand(driveSubsystem::resetOdometry),
        new InstantCommand(() -> driveSubsystem.overwriteInitialOdoPose(PathLoader.getPathInitial(name))),
        PathLoader.loadPath(name, driveSubsystem));
  }

  // public Command getAutonDrive() {
  // return new SequentialCommandGroup(
  // new InstantCommand(driveSubsystem::zeroGyroscope),
  // new InstantCommand(() -> driveSubsystem.setAngle(90)),
  // PathLoader.loadPath("Straight Path", driveSubsystem),
  // new RedAlliance(driveSubsystem),
  // new RunCommand(driveSubsystem::setX, driveSubsystem));
  // }

  public Command middleBalance() {
    return new SequentialCommandGroup(
        new InstantCommand(driveSubsystem::zeroGyroscope),
        new InstantCommand(() -> driveSubsystem.setAngle(90)),
        new RunCommand(() -> driveSubsystem.fieldOrientedDriveNumber(1.0, 0.0, 0.0), driveSubsystem)
            .raceWith(new WaitCommand(1.5)),
        new ABV2Command(driveSubsystem),
        new RunCommand(driveSubsystem::setX, driveSubsystem));
  }

  public Command taxi() {
    return new SequentialCommandGroup(
        new InstantCommand(driveSubsystem::zeroGyroscope),
        new InstantCommand(() -> driveSubsystem.setAngle(90)),
        new RunCommand(() -> driveSubsystem.fieldOrientedDriveNumber(1.5, 0.0, 0.0), driveSubsystem)
            .raceWith(new WaitCommand(4)));
  }

  public Command getWheelPos() {
    return new RunCommand(neoTestSub::output, neoTestSub);
  }

}
