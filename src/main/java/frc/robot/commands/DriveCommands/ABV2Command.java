// Author: UMN Robotics Ri3d
// Last Updated : January 2023

package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

// This command self=balances on the charging station using gyroscope pitch as feedback
public class ABV2Command extends CommandBase {

  private DriveSubsystem m_DriveSubsystem;

  private double error;
  private double currentAngle;
  private double drivePower;
  private Timer timer = new Timer();

  /**
   * Command to use Gyro data to resist the tip angle from the beam - to stabalize
   * and balanace
   */
  public ABV2Command(DriveSubsystem dSub) {
    m_DriveSubsystem = dSub;
    addRequirements(m_DriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Uncomment the line below this to simulate the gyroscope axis with a
    // controller joystick
    // Double currentAngle = -1 *
    // Robot.controller.getRawAxis(Constants.LEFT_VERTICAL_JOYSTICK_AXIS) * 45;
    this.currentAngle = Math.hypot(m_DriveSubsystem.getPitch(), m_DriveSubsystem.getRoll()) - 4.4;
    SmartDashboard.putNumber("RollPitch", currentAngle);
    //Smooth = .05
    //Bad = .1
    error = 0 - currentAngle;
    drivePower = -Math.min(.08 * error, 1);

    // Our robot needed an extra push to drive up in reverse, probably due to weight
    // imbalances

    if (drivePower < 0) {
      drivePower *= 2;
    }
    // drivePower *= 2;

    // Limit the max power
    if (Math.abs(drivePower) > .8) {
      drivePower = Math.copySign(0.8, drivePower);
    }

    m_DriveSubsystem.fieldOrientedDriveNumber(drivePower, 0.0, 0.0);

    SmartDashboard.putNumber("Current Angle", currentAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveSubsystem.stop();
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(error) < 1; // End the command when we are within the specified threshold of being 'flat'
    // return false;                           // (gyroscope pitch of 0 degrees)
    // return false;
  }
}