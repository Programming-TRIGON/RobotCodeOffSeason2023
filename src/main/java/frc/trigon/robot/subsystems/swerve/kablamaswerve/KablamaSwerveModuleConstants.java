package frc.trigon.robot.subsystems.swerve.kablamaswerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.*;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Notifier;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.utilities.Conversions;
import org.littletonrobotics.junction.Logger;

public class KablamaSwerveModuleConstants {
    static final double VOLTAGE_COMPENSATION_SATURATION = 12;
    static final double DRIVE_GEAR_RATIO = 5.14;
    static final double WHEEL_DIAMETER_METERS = 0.1016;
    static final double MAX_THEORETICAL_SPEED_METERS_PER_SECOND = 5.2;

    static final int
            FRONT_LEFT_ID = 0,
            FRONT_RIGHT_ID = 1,
            REAR_LEFT_ID = 2,
            REAR_RIGHT_ID = 3;

    private static final int
            FRONT_LEFT_DRIVE_MOTOR_ID = FRONT_LEFT_ID + 1,
            FRONT_RIGHT_DRIVE_MOTOR_ID = FRONT_RIGHT_ID + 1,
            REAR_LEFT_DRIVE_MOTOR_ID = REAR_LEFT_ID + 1,
            REAR_RIGHT_DRIVE_MOTOR_ID = REAR_RIGHT_ID + 1;
    private static final InvertedValue DRIVE_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive;
    private static final double
            DRIVE_OPEN_LOOP_RAMP_RATE = 0.3,
            DRIVE_CLOSED_LOOP_RAMP_RATE = 0;
    private static final int DRIVE_MOTOR_CURRENT_LIMIT = 100;
    static final SimpleMotorFeedforward DRIVE_FEEDFORWARD = new SimpleMotorFeedforward(0.27202, 1.85, 0.17543);
    static final boolean DRIVE_MOTOR_FOC = false;
    private static final TalonFX
            FRONT_LEFT_DRIVE_MOTOR = new TalonFX(
                    FRONT_LEFT_DRIVE_MOTOR_ID
            ),
            FRONT_RIGHT_DRIVE_MOTOR = new TalonFX(
                    FRONT_RIGHT_DRIVE_MOTOR_ID
            ),
            REAR_LEFT_DRIVE_MOTOR = new TalonFX(
                    REAR_LEFT_DRIVE_MOTOR_ID
            ),
            REAR_RIGHT_DRIVE_MOTOR = new TalonFX(
                    REAR_RIGHT_DRIVE_MOTOR_ID
            );

    private static final int
            FRONT_LEFT_STEER_MOTOR_ID = FRONT_LEFT_ID + 1,
            FRONT_RIGHT_STEER_MOTOR_ID = FRONT_RIGHT_ID + 1,
            REAR_LEFT_STEER_MOTOR_ID = REAR_LEFT_ID + 1,
            REAR_RIGHT_STEER_MOTOR_ID = REAR_RIGHT_ID + 1;
    private static final boolean STEER_MOTOR_INVERTED = false;
    private static final int STEER_MOTOR_CURRENT_LIMIT = 30;
    private static final double
            STEER_MOTOR_P = 0.03,
            STEER_MOTOR_I = 0,
            STEER_MOTOR_D = 0;
    private static final CANSparkMax.IdleMode STEER_MOTOR_IDLE_MODE = CANSparkMax.IdleMode.kBrake;
    private static final CANSparkMax
            FRONT_LEFT_STEER_MOTOR = new CANSparkMax(
                    FRONT_LEFT_STEER_MOTOR_ID,
                    CANSparkMaxLowLevel.MotorType.kBrushless
            ),
            FRONT_RIGHT_STEER_MOTOR = new CANSparkMax(
                    FRONT_RIGHT_STEER_MOTOR_ID,
                    CANSparkMaxLowLevel.MotorType.kBrushless
            ),
            REAR_LEFT_STEER_MOTOR = new CANSparkMax(
                    REAR_LEFT_STEER_MOTOR_ID,
                    CANSparkMaxLowLevel.MotorType.kBrushless
            ),
            REAR_RIGHT_STEER_MOTOR = new CANSparkMax(
                    REAR_RIGHT_STEER_MOTOR_ID,
                    CANSparkMaxLowLevel.MotorType.kBrushless
            );
    private static final KablamaSwerveModuleConstants
            FRONT_LEFT_SWERVE_MODULE_CONSTANTS = new KablamaSwerveModuleConstants(
                    FRONT_LEFT_DRIVE_MOTOR,
                    FRONT_LEFT_STEER_MOTOR
            ),
            FRONT_RIGHT_SWERVE_MODULE_CONSTANTS = new KablamaSwerveModuleConstants(
                    FRONT_RIGHT_DRIVE_MOTOR,
                    FRONT_RIGHT_STEER_MOTOR
            ),
            REAR_LEFT_SWERVE_MODULE_CONSTANTS = new KablamaSwerveModuleConstants(
                    REAR_LEFT_DRIVE_MOTOR,
                    REAR_LEFT_STEER_MOTOR
            ),
            REAR_RIGHT_SWERVE_MODULE_CONSTANTS = new KablamaSwerveModuleConstants(
                    REAR_RIGHT_DRIVE_MOTOR,
                    REAR_RIGHT_STEER_MOTOR
            );

    private static final Translation2d
            FRONT_LEFT_MODULE_LOCATION = new Translation2d(
                    KablamaSwerveConstants.DISTANCE_FROM_CENTER_OF_BASE,
                    KablamaSwerveConstants.DISTANCE_FROM_CENTER_OF_BASE
            ),
            FRONT_RIGHT_MODULE_LOCATION = new Translation2d(
                    KablamaSwerveConstants.DISTANCE_FROM_CENTER_OF_BASE,
                    -KablamaSwerveConstants.DISTANCE_FROM_CENTER_OF_BASE
            ),
            REAR_LEFT_MODULE_LOCATION = new Translation2d(
                    -KablamaSwerveConstants.DISTANCE_FROM_CENTER_OF_BASE,
                    KablamaSwerveConstants.DISTANCE_FROM_CENTER_OF_BASE
            ),
            REAR_RIGHT_MODULE_LOCATION = new Translation2d(
                    -KablamaSwerveConstants.DISTANCE_FROM_CENTER_OF_BASE,
                    -KablamaSwerveConstants.DISTANCE_FROM_CENTER_OF_BASE
            );

    final TalonFX driveMotor;
    final CANSparkMax steerMotor;

    public KablamaSwerveModuleConstants(TalonFX driveMotor, CANSparkMax steerMotor) {
        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;

        if (!RobotConstants.IS_REPLAY) {
            configureDriveMotor(10);
            configureSteerMotor(10);
        }
    }

    private void configureSteerMotor(int repetition) {
        Logger.getInstance().recordOutput("Swerve/" + steerMotor.getDeviceId() + " steerRep", repetition);
        if (steerMotor.restoreFactoryDefaults() != REVLibError.kOk)
            configureSteerMotor(repetition - 1);

        steerMotor.setInverted(STEER_MOTOR_INVERTED);
        if (steerMotor.enableVoltageCompensation(VOLTAGE_COMPENSATION_SATURATION) != REVLibError.kOk)
            configureSteerMotor(repetition - 1);

        steerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 255); // Applied output
        steerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 255); // Motor movement
        steerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 1000); // Motor position
        steerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 1000); // Analog sensor
        steerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus4, 1000); // Alternate encoder
        steerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus5, 20); // Duty cycle position
        steerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus6, 200); // Duty cycle velocity

        final SparkMaxPIDController pidController = steerMotor.getPIDController();
        if (pidController.setP(STEER_MOTOR_P) != REVLibError.kOk)
            configureSteerMotor(repetition - 1);
        if (pidController.setI(STEER_MOTOR_I) != REVLibError.kOk)
            configureSteerMotor(repetition - 1);
        if (pidController.setD(STEER_MOTOR_D) != REVLibError.kOk)
            configureSteerMotor(repetition - 1);
        if (pidController.setPositionPIDWrappingEnabled(true) != REVLibError.kOk)
            configureSteerMotor(repetition - 1);
        if (pidController.setPositionPIDWrappingMinInput(0) != REVLibError.kOk)
            configureSteerMotor(repetition - 1);
        if (pidController.setPositionPIDWrappingMaxInput(Conversions.DEGREES_PER_REVOLUTIONS) != REVLibError.kOk)
            configureSteerMotor(repetition - 1);
        if (pidController.setFeedbackDevice(steerMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)) != REVLibError.kOk)
            configureSteerMotor(repetition - 1);
        if (steerMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).setPositionConversionFactor(Conversions.DEGREES_PER_REVOLUTIONS) != REVLibError.kOk)
            configureSteerMotor(repetition - 1);
        // TODO: current limit
        if (steerMotor.setSmartCurrentLimit(STEER_MOTOR_CURRENT_LIMIT, STEER_MOTOR_CURRENT_LIMIT, 50) != REVLibError.kOk)
            configureSteerMotor(repetition - 1);
        if (steerMotor.setIdleMode(STEER_MOTOR_IDLE_MODE) != REVLibError.kOk)
            configureSteerMotor(repetition - 1);

        new Notifier(steerMotor::burnFlash).startSingle(0.4);
    }

    private void configureDriveMotor(int repetition) {
        Logger.getInstance().recordOutput("Swerve/" + driveMotor.getDeviceID() + " driveRep", repetition);
        final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        motorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = DRIVE_CLOSED_LOOP_RAMP_RATE;
        motorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = DRIVE_OPEN_LOOP_RAMP_RATE;
        motorConfig.MotorOutput.Inverted = DRIVE_MOTOR_INVERTED_VALUE;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.CurrentLimits.SupplyCurrentLimit = DRIVE_MOTOR_CURRENT_LIMIT;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.Audio.BeepOnBoot = false;

        if (driveMotor.getConfigurator().apply(motorConfig) != StatusCode.OK)
            configureDriveMotor(repetition - 1);

        driveMotor.getPosition().setUpdateFrequency(100);
        driveMotor.getVelocity().setUpdateFrequency(20);
        driveMotor.getDutyCycle().setUpdateFrequency(8);
        // TODO: status signals
    }

    enum KablamaSwerveModules {
        FRONT_LEFT(FRONT_LEFT_ID, FRONT_LEFT_SWERVE_MODULE_CONSTANTS, FRONT_LEFT_MODULE_LOCATION),
        FRONT_RIGHT(FRONT_RIGHT_ID, FRONT_RIGHT_SWERVE_MODULE_CONSTANTS, FRONT_RIGHT_MODULE_LOCATION),
        REAR_LEFT(REAR_LEFT_ID, REAR_LEFT_SWERVE_MODULE_CONSTANTS, REAR_LEFT_MODULE_LOCATION),
        REAR_RIGHT(REAR_RIGHT_ID, REAR_RIGHT_SWERVE_MODULE_CONSTANTS, REAR_RIGHT_MODULE_LOCATION);

        final int id;
        final KablamaSwerveModuleConstants swerveModuleConstants;
        final Translation2d location;

        KablamaSwerveModules(int id, KablamaSwerveModuleConstants swerveModuleConstants, Translation2d location) {
            this.id = id;
            this.swerveModuleConstants = swerveModuleConstants;
            this.location = location;
        }

        static KablamaSwerveModules fromId(int id) {
            for (KablamaSwerveModules module : values()) {
                if (module.id == id) {
                    return module;
                }
            }

            throw new IndexOutOfBoundsException("No module with id " + id + " exists");
        }
    }
}

