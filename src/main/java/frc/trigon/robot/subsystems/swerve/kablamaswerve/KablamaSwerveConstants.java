package frc.trigon.robot.subsystems.swerve.kablamaswerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.swerve.SwerveConstants;
import frc.trigon.robot.subsystems.swerve.SwerveModuleIO;

public class KablamaSwerveConstants extends SwerveConstants {
    private static final double RATE_LIMIT = 10;
    static final SlewRateLimiter
            X_SLEW_RATE_LIMITER = new SlewRateLimiter(RATE_LIMIT),
            Y_SLEW_RATE_LIMITER = new SlewRateLimiter(RATE_LIMIT);
    private static final double BRAKE_TIME_SECONDS = 4;
    private static final double
            MAX_SPEED_METERS_PER_SECOND = 4.25,
            MAX_ROTATIONAL_SPEED_RADIANS_PER_SECOND = 12.03;
    private static final double
            DRIVE_NEUTRAL_DEADBAND = 0.1,
            ROTATION_NEUTRAL_DEADBAND = 0.1;
    static final double
            SIDE_LENGTH_METERS = 0.4857,
            DISTANCE_FROM_CENTER_OF_BASE = SIDE_LENGTH_METERS / 2;
    private static final Translation2d[] LOCATIONS = {
            KablamaSwerveModuleConstants.KablamaSwerveModules.fromId(0).location,
            KablamaSwerveModuleConstants.KablamaSwerveModules.fromId(1).location,
            KablamaSwerveModuleConstants.KablamaSwerveModules.fromId(2).location,
            KablamaSwerveModuleConstants.KablamaSwerveModules.fromId(3).location
    };
    static final KablamaSwerveModuleIO[] MODULES_IO = {
            new KablamaSwerveModuleIO(KablamaSwerveModuleConstants.KablamaSwerveModules.fromId(0)),
            new KablamaSwerveModuleIO(KablamaSwerveModuleConstants.KablamaSwerveModules.fromId(1)),
            new KablamaSwerveModuleIO(KablamaSwerveModuleConstants.KablamaSwerveModules.fromId(2)),
            new KablamaSwerveModuleIO(KablamaSwerveModuleConstants.KablamaSwerveModules.fromId(3))
    };
    private static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(LOCATIONS);
    private static final PIDConstants
            TRANSLATION_PID_CONSTANTS = new PIDConstants(13, 0, 0),
            ROTATION_PID_CONSTANTS = new PIDConstants(3, 0, 0),
            AUTO_ROTATION_PID_CONSTANTS = new PIDConstants(6, 0, 0);
//            AUTO_ROTATION_PID_CONSTANTS = new PIDConstants(3,0,0);
    private static final int PIGEON_ID = 0;
    static final Pigeon2 GYRO = new Pigeon2(PIGEON_ID);
    private static final double DEGREES_TOLERANCE = 2;
    private static final TrapezoidProfile.Constraints
            ROTATION_CONSTRAINTS = new TrapezoidProfile.Constraints(
                    720,
                    1200
            ),
            TRANSLATION_CONSTRAINTS = new TrapezoidProfile.Constraints(
                    3,
                    3
            );

    private static final PIDController ROTATION_PID_CONTROLLER = new PIDController(
            3, 0, 0
    );

    private static final ProfiledPIDController
            PROFILED_PID_CONTROLLER = new ProfiledPIDController(
                    ROTATION_PID_CONSTANTS.kP,
                    ROTATION_PID_CONSTANTS.kI,
                    ROTATION_PID_CONSTANTS.kD,
                    ROTATION_CONSTRAINTS
            ),
            PROFILED_Y_AXIS_CONTROLLER = new ProfiledPIDController(
                    TRANSLATION_PID_CONSTANTS.kP,
                    TRANSLATION_PID_CONSTANTS.kI,
                    TRANSLATION_PID_CONSTANTS.kD,
                    TRANSLATION_CONSTRAINTS
            );
    private static final double
            TRANSLATION_TOLERANCE = 0.2,
            ROTATION_TOLERANCE = 1,
            TRANSLATION_VELOCITY_TOLERANCE = 0.05,
            ROTATION_VELOCITY_TOLERANCE = 0.05;

    static StatusSignal<Double> YAW_SIGNAL, PITCH_SIGNAL, ROLL_SIGNAL, PITCH_VELOCITY_SIGNAL, ROLL_VELOCITY_SIGNAL, X_ACCELERATION_SIGNAL, Y_ACCELERATION_SIGNAL, Z_ACCELERATION_SIGNAL;

    static {
        PROFILED_PID_CONTROLLER.enableContinuousInput(-180, 180);
        ROTATION_PID_CONTROLLER.enableContinuousInput(-180, 180);
        PROFILED_PID_CONTROLLER.setTolerance(DEGREES_TOLERANCE);
        PROFILED_PID_CONTROLLER.setIntegratorRange(-30, 30);

        if (!RobotConstants.IS_REPLAY)
            configureGyro();
    }

    private static void configureGyro() {
        GYRO.getConfigurator().apply(new Pigeon2Configuration());

        YAW_SIGNAL = GYRO.getYaw();
        PITCH_SIGNAL = GYRO.getPitch();
        ROLL_SIGNAL = GYRO.getRoll();
        X_ACCELERATION_SIGNAL = GYRO.getAccelerationX();
        Y_ACCELERATION_SIGNAL = GYRO.getAccelerationY();
        Z_ACCELERATION_SIGNAL = GYRO.getAccelerationZ();
        PITCH_VELOCITY_SIGNAL = GYRO.getAngularVelocityX();
        ROLL_VELOCITY_SIGNAL = GYRO.getAngularVelocityY();

        YAW_SIGNAL.setUpdateFrequency(100);
        PITCH_SIGNAL.setUpdateFrequency(30);
        ROLL_SIGNAL.setUpdateFrequency(30);
        PITCH_VELOCITY_SIGNAL.setUpdateFrequency(30);
        ROLL_VELOCITY_SIGNAL.setUpdateFrequency(30);
        X_ACCELERATION_SIGNAL.setUpdateFrequency(10);
        Y_ACCELERATION_SIGNAL.setUpdateFrequency(4);
        Z_ACCELERATION_SIGNAL.setUpdateFrequency(4);

        GYRO.getAngularVelocityX().setUpdateFrequency(4);
        GYRO.getAngularVelocityY().setUpdateFrequency(4);
        GYRO.getAngularVelocityZ().setUpdateFrequency(4);
        GYRO.getGravityVectorX().setUpdateFrequency(4);
        GYRO.getGravityVectorY().setUpdateFrequency(4);
        GYRO.getGravityVectorZ().setUpdateFrequency(4);
    }

    @Override
    protected SwerveModuleIO[] getModulesIO() {
        return MODULES_IO;
    }

    @Override
    public SwerveDriveKinematics getKinematics() {
        return KINEMATICS;
    }

    @Override
    protected double getDriveNeutralDeadband() {
        return DRIVE_NEUTRAL_DEADBAND;
    }

    @Override
    protected double getRotationNeutralDeadband() {
        return ROTATION_NEUTRAL_DEADBAND;
    }

    @Override
    public PIDConstants getTranslationPIDConstants() {
        return TRANSLATION_PID_CONSTANTS;
    }

    @Override
    protected PIDConstants getRotationPIDConstants() {
        return ROTATION_PID_CONSTANTS;
    }

    @Override
    protected PIDConstants getAutoRotationPIDConstants() {
        return AUTO_ROTATION_PID_CONSTANTS;
    }

    @Override
    protected double getMaxSpeedMetersPerSecond() {
        return MAX_SPEED_METERS_PER_SECOND;
    }

    @Override
    protected double getMaxRotationalSpeedRadiansPerSecond() {
        return MAX_ROTATIONAL_SPEED_RADIANS_PER_SECOND;
    }

    @Override
    protected double getBrakeTimeSeconds() {
        return BRAKE_TIME_SECONDS;
    }

    @Override
    protected PIDController getRotationController() {
        return ROTATION_PID_CONTROLLER;
    }

    @Override
    protected ProfiledPIDController getProfiledYAxisController() {
        return PROFILED_Y_AXIS_CONTROLLER;
    }

    @Override
    public ProfiledPIDController getProfiledRotationController() {
        return PROFILED_PID_CONTROLLER;
    }

    @Override
    protected double getTranslationTolerance() {
        return TRANSLATION_TOLERANCE;
    }

    @Override
    protected double getRotationTolerance() {
        return ROTATION_TOLERANCE;
    }

    @Override
    protected double getTranslationVelocityTolerance() {
        return TRANSLATION_VELOCITY_TOLERANCE;
    }

    @Override
    protected double getRotationVelocityTolerance() {
        return ROTATION_VELOCITY_TOLERANCE;
    }

    @Override
    public double getRobotSideLength() {
        return SIDE_LENGTH_METERS;
    }

    @Override
    protected Translation2d[] getModuleLocations() {
        return LOCATIONS;
    }

    @Override
    protected SlewRateLimiter getXSlewRateLimiter() {
        return X_SLEW_RATE_LIMITER;
    }

    @Override
    protected SlewRateLimiter getYSlewRateLimiter() {
        return Y_SLEW_RATE_LIMITER;
    }
}
