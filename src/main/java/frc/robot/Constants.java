// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public final static double L = .737;
  public final static double W = .737;

  public final static SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      // FL
      new Translation2d(L / 2, W / 2), // .4041 .5751
      // FR
      new Translation2d(L / 2, -W / 2),
      // BL
      new Translation2d(-L / 2, -W / 2),
      // BR
      new Translation2d(-L / 2, W / 2));

  public static final double WRIST_MAX = 360;
  public static final double ARM_MAX = 360;
  public static final double EXTEND_MAX = 180;

  public static final int wristID = 44;

  public static final int vacuum99ID = 99;
  public static final int vacuum88ID = 88;
  public static final int vacuum77ID = 77;

  public static final int armExtendID = 33;

  public static final int armAngle11ID = 11;
  public static final int armAngle22ID = 22;

  public static final int wristRotID = 55;

  public static final double NOMINAL_VOLTAGE = 12.3;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
