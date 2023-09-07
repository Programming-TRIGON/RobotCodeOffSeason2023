package frc.trigon.robot.subsystems.sideshooter.kablamasideshooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.sideshooter.SideShooterConstants;

public class KablamaSideShooterConstants extends SideShooterConstants {
    static final double VOLTAGE_COMPENSATION_SATURATION = 12;
    static final boolean SHOOTING_MOTOR_FOC = false;

    private static final boolean ANGLE_MOTOR_INVERTED = false;
    private static final InvertedValue SHOOTING_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    private static final int
            SHOOTING_MOTOR_ID = 1,
            ANGLE_MOTOR_ID = 1;
    private static final NeutralModeValue DEFAULT_SHOOTING_MOTOR_NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    private static final CANSparkMax.IdleMode DEFAULT_ANGLE_MOTOR_IDLE_MODE = CANSparkMax.IdleMode.kBrake;
    private static final double
            ANGLE_MOTOR_P = 1,
            ANGLE_MOTOR_I = 0,
            ANGLE_MOTOR_D = 0;
    private static final double ANGLE_ENCODER_OFFSET = 0;
    private static final int
            ANGLE_MOTOR_CURRENT_LIMIT = 30,
            SHOOTING_MOTOR_CURRENT_LIMIT = 30;

    static final TalonFX SHOOTING_MOTOR = new TalonFX(SHOOTING_MOTOR_ID);
    static final CANSparkMax ANGLE_MOTOR = new CANSparkMax(ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    static final SparkMaxAbsoluteEncoder ANGLE_ENCODER = ANGLE_MOTOR.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

    static StatusSignal<Double> STATOR_CURRENT_SIGNAL, MOTOR_OUTPUT_PERCENT_SIGNAL;

    private static final double
            ANGLE_MOTOR_KS = 0,
            ANGLE_MOTOR_KV = 0,
            ANGLE_MOTOR_KA = 0,
            ANGLE_MOTOR_KG = 0;
    private static final ArmFeedforward ANGLE_MOTOR_FEEDFORWARD = new ArmFeedforward(
            ANGLE_MOTOR_KS, ANGLE_MOTOR_KG, ANGLE_MOTOR_KV, ANGLE_MOTOR_KA
    );

    static {
        if (!RobotConstants.IS_REPLAY) {
            configureShootingMotor();
            configureAngleMotor();
        }
    }

    @Override
    protected ArmFeedforward getAngleMotorFeedforward() {
        return ANGLE_MOTOR_FEEDFORWARD;
    }

    private static void configureShootingMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = SHOOTING_MOTOR_INVERTED_VALUE;
        config.MotorOutput.NeutralMode = DEFAULT_SHOOTING_MOTOR_NEUTRAL_MODE_VALUE;

        config.CurrentLimits.StatorCurrentLimit = SHOOTING_MOTOR_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        SHOOTING_MOTOR.getConfigurator().apply(config);

        STATOR_CURRENT_SIGNAL = SHOOTING_MOTOR.getStatorCurrent();
        MOTOR_OUTPUT_PERCENT_SIGNAL = SHOOTING_MOTOR.getDutyCycle();

        STATOR_CURRENT_SIGNAL.setUpdateFrequency(10);
        MOTOR_OUTPUT_PERCENT_SIGNAL.setUpdateFrequency(10);
    }

    private static void configureAngleMotor() {
        ANGLE_MOTOR.restoreFactoryDefaults();

        ANGLE_MOTOR.enableVoltageCompensation(VOLTAGE_COMPENSATION_SATURATION);
        ANGLE_MOTOR.setInverted(ANGLE_MOTOR_INVERTED);
        ANGLE_MOTOR.setIdleMode(DEFAULT_ANGLE_MOTOR_IDLE_MODE);
        ANGLE_MOTOR.setSmartCurrentLimit(ANGLE_MOTOR_CURRENT_LIMIT);

        ANGLE_MOTOR.getPIDController().setP(ANGLE_MOTOR_P);
        ANGLE_MOTOR.getPIDController().setI(ANGLE_MOTOR_I);
        ANGLE_MOTOR.getPIDController().setD(ANGLE_MOTOR_D);
        ANGLE_MOTOR.getPIDController().setPositionPIDWrappingEnabled(true);

        ANGLE_ENCODER.setPositionConversionFactor(360);
        ANGLE_ENCODER.setVelocityConversionFactor(360);
        ANGLE_ENCODER.setZeroOffset(ANGLE_ENCODER_OFFSET);
        ANGLE_MOTOR.getPIDController().setFeedbackDevice(ANGLE_ENCODER);

        ANGLE_MOTOR.burnFlash();
    }
}
