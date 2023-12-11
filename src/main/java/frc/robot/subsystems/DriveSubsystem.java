// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.CycloidLibrary.CycloidCalculations;
import frc.robot.CycloidLibrary.NeoSteveModule;
//import frc.robot.CycloidLibrary.CycloidCalculations;
import frc.robot.CycloidLibrary.SteveModule;

public class DriveSubsystem extends SubsystemBase {
  final String CAN = "OTHERCANIVORE";
  WPI_Pigeon2 pidgey = new WPI_Pigeon2(10);
  NeoSteveModule fl, fr, br, bl;
  double floffset = -3.021936044096947 + Math.PI;
  double froffset = -2.64150957763195;
  double broffset = -2.012578725814819;
  double bloffset = 2.376131437718868 + Math.PI;

  double desiredHeading = pidgey.getAngle();
  PIDController headingController = new PIDController(-.0001, 0.0, 0.000001);

  SwerveDriveOdometry odometry;
  Field2d field;

  public DriveSubsystem() {
    fl = new NeoSteveModule(1, 2, 50, floffset);
    fr = new NeoSteveModule(3, 4, 51, froffset);
    br = new NeoSteveModule(5, 6, 53, broffset);
    bl = new NeoSteveModule(7, 8, 52, bloffset);

    // fl.ramp(.5);
    // fr.ramp(.5);
    // br.ramp(.5);
    // bl.ramp(.5);

    odometry = new SwerveDriveOdometry(Constants.m_kinematics, getAngle(), getModulePositions());
    field = new Field2d();
    resetOdometry();
    SmartDashboard.putData("fiedd", field);
  }

  public Pose2d getOdoPose() {
    Pose2d unchanged = odometry.getPoseMeters();
    return new Pose2d(unchanged.getX(), unchanged.getY(), unchanged.getRotation());
  }

  public void overwriteInitialOdoPose(Pose2d newPose) {
    odometry.resetPosition(newPose.getRotation(), getModulePositions(), newPose);
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees((pidgey.getAngle() % 360));
    // return new Rotation2d();
  }

  public void setAutonSwerveModuleStates(SwerveModuleState[] states) {
    ChassisSpeeds sNew = Constants.m_kinematics.toChassisSpeeds(states);
    sNew.vxMetersPerSecond *= -1;
    setSwerveModuleStates(Constants.m_kinematics.toSwerveModuleStates(sNew));
  }

  public void setSwerveModuleStates(SwerveModuleState[] states) {
    // bl.setModuleState(new SwerveModuleState(0, Rotation2d.fromRadians(Math.PI /
    // 2)));
    bl.setModuleState(states[0]);
    br.setModuleState(states[1]);
    fr.setModuleState(states[2]);
    fl.setModuleState(states[3]);
  }

  public void displaySwerveModuleState(SwerveModuleState s, String name) {
    SmartDashboard.putNumber(name + "_STEER", s.angle.getDegrees());
    SmartDashboard.putNumber(name + "_DRIVE", s.speedMetersPerSecond);
  }

  public void drive(ChassisSpeeds speeds) {
    // SwerveModuleState[] cycloidStates =
    // CycloidCalculations.toSwerveModuleStates(speeds,
    // getYaw());
    SwerveModuleState[] kinematicStates = Constants.m_kinematics.toSwerveModuleStates(speeds);

    // displaySwerveModuleState(cycloidStates[0], "CYCLOIDSTATES");
    // displaySwerveModuleState(kinematicStates[0], "KINEMATICSSTATES");
    setSwerveModuleStates(kinematicStates);
  }

  public ChassisSpeeds getAdjustedChassisSpeeds(ChassisSpeeds oldSpeeds) { // Adjusts chassisSpeeds to closed loop
                                                                           // heading control setpoint
    double offset = headingController.calculate(pidgey.getAngle(), desiredHeading);
    oldSpeeds.omegaRadiansPerSecond = offset;
    return oldSpeeds;
  }

  public void adjustedDrive(ChassisSpeeds noRotSpeeds, double controllerInput) { // ControllerInput should be between -1
                                                                                 // & 1
    desiredHeading += controllerInput;
    drive(getAdjustedChassisSpeeds(noRotSpeeds));
  }

  public void fieldOrientedDriveNumber(Double xtrans, Double ytrans, Double rot) {
    drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xtrans,
            ytrans,
            rot,
            getAngle()));
  }

  public void driveFromNum(Double xtrans, Double ytrans, Double rot) {
    drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xtrans,
            ytrans,
            rot,
            getAngle()));
  }

  public void stop() {
    driveFromNum(0.0, 0.0, 0.0);
  }

  public void setX() {
    fl.setModuleState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
    fr.setModuleState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(315.0)));
    bl.setModuleState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(315.0)));
    br.setModuleState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] arr = {
        fl.getSwerveModuleState(),
        fr.getSwerveModuleState(),
        bl.getSwerveModuleState(),
        br.getSwerveModuleState()
    };

    return arr;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] arr = {
        fl.getSwerveModulePosition(),
        fr.getSwerveModulePosition(),
        bl.getSwerveModulePosition(),
        br.getSwerveModulePosition()
    };

    return arr;
  }

  public double getRoll() {
    return pidgey.getRoll();
  }

  public double getPitch() {
    return pidgey.getPitch();
  }

  public void setAngle(double angle) {
    pidgey.addYaw(angle);
  }

  public void zeroGyroscope() {
    pidgey.reset();
    resetOdometry();
    desiredHeading = 0;
  }

  public void resetOdometry() {
    odometry.resetPosition(getAngle(), getModulePositions(), new Pose2d());
  }

  public void offsetYaw(double heading) {
    pidgey.setYaw(pidgey.getYaw() + heading);
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public double getYaw() {
    return pidgey.getYaw();
  }

  public void resetDesiredHeading() {
    desiredHeading = pidgey.getAngle();
  }

  public void log() {
    SmartDashboard.putNumber("FLEFT", fl.getEncoderPosition());
    SmartDashboard.putNumber("FRIGHT", fr.getEncoderPosition());
    SmartDashboard.putNumber("BLEFT", bl.getEncoderPosition());
    SmartDashboard.putNumber("BRIGHT", br.getEncoderPosition());

    SmartDashboard.putNumber("flmps", fl.getSpeedMPS());

    field.setRobotPose(getOdoPose());
    // SmartDashboard.putNumber("FRIGHT Error", fr.getError());
    // SmartDashboard.putNumber("FLEFT Error", fl.getError());
    // SmartDashboard.putNumber("BRIGHT Error", br.getError());
    // SmartDashboard.putNumber("BLEFT Error", bl.getError());

  }

  @Override
  public void periodic() {
    odometry.update(getAngle(), getModulePositions());
    this.log();
  }
}
