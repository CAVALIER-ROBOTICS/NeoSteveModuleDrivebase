
package frc.robot.CycloidLibrary;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.ctre.phoenix.sensors.CANCoder;

public class SteveModule {
    TalonFX drive, steer;
    CANCoder absoluteEncoder;
    PIDController driveController, steerController;
    SimpleMotorFeedforward ff;
    double offset;

    public SteveModule(int drivePort, int steerPort, int encoderPort, double off, String canBus) {
        CANCoder enc = new CANCoder(encoderPort, canBus);
        TalonFX d = new TalonFX(drivePort, canBus);
        TalonFX s = new TalonFX(steerPort, canBus);

        setupEncoder(enc);
        this.absoluteEncoder = enc;
        setupDriveMotor(d);
        this.drive = d;
        setupTurnMotor(s);
        this.steer = s;

        driveController = new PIDController(.01, 0, 0);
        steerController = new PIDController(.3, 0.0, 0.0);
        steerController.enableContinuousInput(-Math.PI, Math.PI);
        ff = new SimpleMotorFeedforward(0.015, .212);

        this.offset = off;
    }

    public SteveModule(int drivePort, int steerPort, int encoderPort, double off) {
        CANCoder enc = new CANCoder(encoderPort);
        TalonFX d = new TalonFX(drivePort);
        TalonFX s = new TalonFX(steerPort);

        setupEncoder(enc);
        this.absoluteEncoder = enc;
        setupDriveMotor(d);
        this.drive = d;
        setupTurnMotor(s);
        this.steer = s;

        driveController = new PIDController(.01, 0, 0);
        steerController = new PIDController(.3, 0.0, 0.0);
        steerController.enableContinuousInput(-Math.PI, Math.PI);
        ff = new SimpleMotorFeedforward(.015, .212);

        this.offset = off;
    }

    public void setModuleState(SwerveModuleState state) {
        double currentMeasurement = coterminal(absoluteEncoder.getAbsolutePosition() - offset);
        state = SwerveModuleState.optimize(state, Rotation2d.fromRadians(currentMeasurement));

        double speed = state.speedMetersPerSecond;
        double theta = state.angle.getRadians();

        double drivePercent = driveController.calculate(drive.getSelectedSensorVelocity(), speed);
        double steerPercent = steerController.calculate(coterminal(absoluteEncoder.getAbsolutePosition() - offset),
                theta);

        drive.set(ControlMode.PercentOutput, drivePercent + ff.calculate(speed));
        steer.set(ControlMode.PercentOutput, steerPercent);
    }

    public SwerveModuleState getSwerveModuleState() {
        return new SwerveModuleState(drive.getSelectedSensorVelocity(), Rotation2d.fromRadians(getEncoderPosition()));
    }

    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(drive.getSelectedSensorPosition(),
                Rotation2d.fromRadians(getEncoderPosition()));
    }

    public void ramp(double rate) {
        this.drive.configOpenloopRamp(rate);
    }

    private static double coterminal(final double rotation) {
        return rotation;

        // double coterminal = rotation;
        // final double full = Math.signum(rotation) * 2 * Math.PI;
        // while (coterminal > Math.PI || coterminal < -Math.PI)
        // coterminal -= full;
        // return coterminal;
    }

    private void setupEncoder(CANCoder encoder) {
        CANCoderConfiguration configPls = new CANCoderConfiguration();
        configPls.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        configPls.sensorCoefficient = 2 * Math.PI / 4096;
        configPls.unitString = "rad";
        configPls.sensorTimeBase = SensorTimeBase.PerSecond;

        encoder.configAllSettings(configPls);
    }

    public double getEncoderPosition() {
        return this.absoluteEncoder.getAbsolutePosition();
    }

    private void setupDriveMotor(TalonFX driveMotor) {
        driveMotor.configFactoryDefault();
        driveMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.setInverted(false);
        driveMotor.configSelectedFeedbackCoefficient((1 / 6.75 / 60) * (.1016 * Math.PI)); // Ez meters per second (NNOT
        driveMotor.enableVoltageCompensation(true);

    }

    private void setupTurnMotor(TalonFX steerMotor) {
        steerMotor.configFactoryDefault();
        steerMotor.setNeutralMode(NeutralMode.Brake);
        steerMotor.setInverted(false);
        steerMotor.configSelectedFeedbackCoefficient(12.8 * 2 * Math.PI);
        steerMotor.enableVoltageCompensation(true);
    }

}
