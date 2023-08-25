package frc.trigon.robot.subsystems.arm.kablamaarm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.arm.ArmConstants;

import java.util.HashMap;

public class KablamaArmConstants extends ArmConstants {
    private static final double VOLTAGE_COMPENSATION_SATURATION = 12;

    private static final int
            MASTER_ANGLE_MOTOR_ID = 0,
            FIRST_FOLLOWER_ANGLE_MOTOR_ID = 0,
            SECOND_FOLLOWER_ANGLE_MOTOR_ID = 0;
    private static final boolean
            MASTER_ANGLE_MOTOR_INVERTED = false,
            FIRST_FOLLOWER_ANGLE_MOTOR_INVERTED = false,
            SECOND_FOLLOWER_ANGLE_MOTOR_INVERTED = false;
    private static final double
            ANGLE_MOTOR_P = 0,
            ANGLE_MOTOR_I = 0,
            ANGLE_MOTOR_D = 0;
    private static final CANSparkMax.IdleMode ANGLE_MOTOR_DEFAULT_NEUTRAL_MODE = CANSparkMax.IdleMode.kBrake;
    private static final double ANGLE_ENCODER_OFFSET = 0;
    private static final int ANGLE_CURRENT_LIMIT = 30;
    static final CANSparkMax MASTER_ANGLE_MOTOR = new CANSparkMax(MASTER_ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    static final SparkMaxAbsoluteEncoder ANGLE_ENCODER = MASTER_ANGLE_MOTOR.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    private static final CANSparkMax
            FIRST_FOLLOWER_ANGLE_MOTOR = new CANSparkMax(FIRST_FOLLOWER_ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless),
            SECOND_FOLLOWER_ANGLE_MOTOR = new CANSparkMax(SECOND_FOLLOWER_ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

    private static final int
            MASTER_ELEVATOR_MOTOR_ID = 0,
            FOLLOWER_ELEVATOR_MOTOR_ID = 0;
    private static final boolean
            MASTER_ELEVATOR_MOTOR_INVERTED = false,
            FOLLOWER_ELEVATOR_MOTOR_INVERTED = false;
    private static final double
            ELEVATOR_MOTOR_P = 0,
            ELEVATOR_MOTOR_I = 0,
            ELEVATOR_MOTOR_D = 0;
    private static final CANSparkMax.IdleMode ELEVATOR_MOTOR_DEFAULT_NEUTRAL_MODE = CANSparkMax.IdleMode.kBrake;
    private static final double ELEVATOR_ENCODER_OFFSET = 0;
    private static final int ELEVATOR_CURRENT_LIMIT = 30;
    static final CANSparkMax MASTER_ELEVATOR_MOTOR = new CANSparkMax(MASTER_ELEVATOR_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    static final SparkMaxAbsoluteEncoder ELEVATOR_ENCODER = MASTER_ELEVATOR_MOTOR.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    private static final CANSparkMax FOLLOWER_ELEVATOR_MOTOR = new CANSparkMax(FOLLOWER_ELEVATOR_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

    private static final double
            ELEVATOR_MOTOR_KS = 0,
            ELEVATOR_MOTOR_KV = 0,
            ELEVATOR_MOTOR_KA = 0,
            ELEVATOR_MOTOR_KG = 0;
    private static final ElevatorFeedforward ELEVATOR_MOTOR_FEEDFORWARD = new ElevatorFeedforward(
            ELEVATOR_MOTOR_KS, ELEVATOR_MOTOR_KG, ELEVATOR_MOTOR_KV, ELEVATOR_MOTOR_KA
    );
    private static final double
            HEIGHT_0_ANGLE_MOTOR_KS = 0,
            HEIGHT_0_ANGLE_MOTOR_KV = 0,
            HEIGHT_0_ANGLE_MOTOR_KA = 0,
            HEIGHT_0_ANGLE_MOTOR_KG = 0,
            HEIGHT_20_ANGLE_MOTOR_KS = 0,
            HEIGHT_20_ANGLE_MOTOR_KV = 0,
            HEIGHT_20_ANGLE_MOTOR_KA = 0,
            HEIGHT_20_ANGLE_MOTOR_KG = 0;
    private static final ArmFeedforward
            HEIGHT_0_ANGLE_MOTOR_FEEDFORWARD = new ArmFeedforward(
                    HEIGHT_0_ANGLE_MOTOR_KS, HEIGHT_0_ANGLE_MOTOR_KG,
                    HEIGHT_0_ANGLE_MOTOR_KV, HEIGHT_0_ANGLE_MOTOR_KA
            ),
            HEIGHT_20_ANGLE_MOTOR_FEEDFORWARD = new ArmFeedforward(
                    HEIGHT_20_ANGLE_MOTOR_KS, HEIGHT_20_ANGLE_MOTOR_KG,
                    HEIGHT_20_ANGLE_MOTOR_KV, HEIGHT_20_ANGLE_MOTOR_KA
            );
    private static final HashMap<Double, ArmFeedforward> HEIGHT_TO_ANGLE_MOTOR_FEEDFORWARD_MAP = new HashMap<>();

    static {
        HEIGHT_TO_ANGLE_MOTOR_FEEDFORWARD_MAP.put(0.0, HEIGHT_0_ANGLE_MOTOR_FEEDFORWARD);
        HEIGHT_TO_ANGLE_MOTOR_FEEDFORWARD_MAP.put(20.0, HEIGHT_20_ANGLE_MOTOR_FEEDFORWARD);

        if (!RobotConstants.IS_REPLAY) {
            configureElevatorMotor();
            configureAngleMotor();
        }
    }

    @Override
    protected HashMap<Double, ArmFeedforward> getHeightToAngleMotorFeedforwardMap() {
        return HEIGHT_TO_ANGLE_MOTOR_FEEDFORWARD_MAP;
    }

    @Override
    protected ElevatorFeedforward getElevatorMotorFeedforward() {
        return ELEVATOR_MOTOR_FEEDFORWARD;
    }

    private static void configureAngleMotor() {
        MASTER_ANGLE_MOTOR.restoreFactoryDefaults();
        FIRST_FOLLOWER_ANGLE_MOTOR.restoreFactoryDefaults();
        SECOND_FOLLOWER_ANGLE_MOTOR.restoreFactoryDefaults();

        MASTER_ANGLE_MOTOR.enableVoltageCompensation(VOLTAGE_COMPENSATION_SATURATION);
        FIRST_FOLLOWER_ANGLE_MOTOR.enableVoltageCompensation(VOLTAGE_COMPENSATION_SATURATION);
        SECOND_FOLLOWER_ANGLE_MOTOR.enableVoltageCompensation(VOLTAGE_COMPENSATION_SATURATION);

        MASTER_ANGLE_MOTOR.setInverted(MASTER_ANGLE_MOTOR_INVERTED);
        FIRST_FOLLOWER_ANGLE_MOTOR.setInverted(FIRST_FOLLOWER_ANGLE_MOTOR_INVERTED);
        SECOND_FOLLOWER_ANGLE_MOTOR.setInverted(SECOND_FOLLOWER_ANGLE_MOTOR_INVERTED);

        MASTER_ANGLE_MOTOR.getPIDController().setP(ANGLE_MOTOR_P);
        MASTER_ANGLE_MOTOR.getPIDController().setI(ANGLE_MOTOR_I);
        MASTER_ANGLE_MOTOR.getPIDController().setD(ANGLE_MOTOR_D);
        MASTER_ANGLE_MOTOR.getPIDController().setPositionPIDWrappingEnabled(true);

        ANGLE_ENCODER.setPositionConversionFactor(360);
        ANGLE_ENCODER.setVelocityConversionFactor(360);
        ANGLE_ENCODER.setZeroOffset(ANGLE_ENCODER_OFFSET);
        MASTER_ANGLE_MOTOR.getPIDController().setFeedbackDevice(ANGLE_ENCODER);

        FIRST_FOLLOWER_ANGLE_MOTOR.follow(MASTER_ANGLE_MOTOR);
        SECOND_FOLLOWER_ANGLE_MOTOR.follow(MASTER_ANGLE_MOTOR);

        MASTER_ANGLE_MOTOR.setIdleMode(ANGLE_MOTOR_DEFAULT_NEUTRAL_MODE);
        FIRST_FOLLOWER_ANGLE_MOTOR.setIdleMode(ANGLE_MOTOR_DEFAULT_NEUTRAL_MODE);
        SECOND_FOLLOWER_ANGLE_MOTOR.setIdleMode(ANGLE_MOTOR_DEFAULT_NEUTRAL_MODE);

        MASTER_ANGLE_MOTOR.setSmartCurrentLimit(ANGLE_CURRENT_LIMIT);
        FIRST_FOLLOWER_ANGLE_MOTOR.setSmartCurrentLimit(ANGLE_CURRENT_LIMIT);
        SECOND_FOLLOWER_ANGLE_MOTOR.setSmartCurrentLimit(ANGLE_CURRENT_LIMIT);

        // TODO: Periodic Frame Periods

        MASTER_ANGLE_MOTOR.burnFlash();
        FIRST_FOLLOWER_ANGLE_MOTOR.burnFlash();
        SECOND_FOLLOWER_ANGLE_MOTOR.burnFlash();
    }

    private static void configureElevatorMotor() {
        MASTER_ELEVATOR_MOTOR.restoreFactoryDefaults();
        FOLLOWER_ELEVATOR_MOTOR.restoreFactoryDefaults();

        MASTER_ELEVATOR_MOTOR.enableVoltageCompensation(VOLTAGE_COMPENSATION_SATURATION);
        FOLLOWER_ELEVATOR_MOTOR.enableVoltageCompensation(VOLTAGE_COMPENSATION_SATURATION);

        MASTER_ELEVATOR_MOTOR.setInverted(MASTER_ELEVATOR_MOTOR_INVERTED);
        FOLLOWER_ELEVATOR_MOTOR.setInverted(FOLLOWER_ELEVATOR_MOTOR_INVERTED);

        MASTER_ELEVATOR_MOTOR.getPIDController().setP(ELEVATOR_MOTOR_P);
        MASTER_ELEVATOR_MOTOR.getPIDController().setI(ELEVATOR_MOTOR_I);
        MASTER_ELEVATOR_MOTOR.getPIDController().setD(ELEVATOR_MOTOR_D);
        MASTER_ELEVATOR_MOTOR.getPIDController().setPositionPIDWrappingEnabled(true);

        ELEVATOR_ENCODER.setZeroOffset(ELEVATOR_ENCODER_OFFSET);
        MASTER_ELEVATOR_MOTOR.getPIDController().setFeedbackDevice(ELEVATOR_ENCODER);
        FOLLOWER_ELEVATOR_MOTOR.follow(MASTER_ELEVATOR_MOTOR);

        MASTER_ELEVATOR_MOTOR.setIdleMode(ELEVATOR_MOTOR_DEFAULT_NEUTRAL_MODE);
        FOLLOWER_ELEVATOR_MOTOR.setIdleMode(ELEVATOR_MOTOR_DEFAULT_NEUTRAL_MODE);

        MASTER_ELEVATOR_MOTOR.setSmartCurrentLimit(ELEVATOR_CURRENT_LIMIT);
        FOLLOWER_ELEVATOR_MOTOR.setSmartCurrentLimit(ELEVATOR_CURRENT_LIMIT);

        // TODO: Periodic Frame Periods

        MASTER_ELEVATOR_MOTOR.burnFlash();
        FOLLOWER_ELEVATOR_MOTOR.burnFlash();
    }
}
