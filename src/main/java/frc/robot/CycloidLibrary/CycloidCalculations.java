package frc.robot.CycloidLibrary;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CycloidCalculations {
    public static final double r = 0.3685; // meters
    public static final double[][] steveLocations = {
            { -r, r }, // FL
            { r, r }, // FR
            { r, -r }, // BR
            { -r, -r } }; // BL

    // takes generated states array and converts it into proper object data type
    public static SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds speeds, double currentYaw) {
        double[] arr = generateSteves(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond,
                currentYaw);
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            double speedMPS = arr[i];
            Rotation2d theta = Rotation2d.fromRadians(arr[i + 4]);
            SmartDashboard.putNumber(String.valueOf(i), speedMPS);
            states[i] = new SwerveModuleState(speedMPS, theta);
        }
        return states;
    }

    // takes chassis speeds and a yaw then generates an array of motor setpoints
    // using the 3 methods below
    private static double[] generateSteves(double vx, double vy, double vr, double yaw) {
        double[] steve = new double[8];
        for (byte i = 0; i < 4; i++) {
            steve[i] = D(V(steveLocations[i], vx, vy, vr, yaw));
        }
        for (byte i = 0; i < 4; i++) {
            steve[i + 4] = S(V(steveLocations[i], vx, vy, vr, yaw), yaw);
        }
        return steve;
    }

    // takes a steve position, chassis speeds, and a yaw then outputs a velocity
    // vector
    public static double[] V(double[] p, double vx, double vy, double vr, double yaw) {
        double[] velocityVector = new double[2];
        velocityVector[0] = vx - vr * (p[1] * Math.cos(yaw) + p[0] * Math.sin(yaw));
        velocityVector[1] = vy + vr * (p[0] * Math.cos(yaw) - p[1] * Math.sin(yaw));
        return velocityVector;
    }

    // Takes a vector and a yaw then outputs a vector angle (used for steer motors)
    public static double S(double[] v, double yaw) {
        double sangle = (Math.atan2(v[0], v[1]) - yaw) % (2 * Math.PI);
        return Math.PI / 2 - sangle;
    }

    // Takes a vector then outputs a vector magnitude (used for drive motors)
    public static double D(double[] v) {
        double dagnitude = Math.hypot(v[0], v[1]);
        return dagnitude;
    }
}
