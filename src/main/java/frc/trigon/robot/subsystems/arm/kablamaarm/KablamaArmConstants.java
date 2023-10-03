package frc.trigon.robot.subsystems.arm.kablamaarm;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Notifier;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.arm.ArmConstants;
import frc.trigon.robot.utilities.Conversions;
import frc.trigon.robot.utilities.SRXMagEncoder;

import java.util.HashMap;
import java.util.List;

public class KablamaArmConstants extends ArmConstants {
    private static final double VOLTAGE_COMPENSATION_SATURATION = 12;

    private static final int
            MASTER_ANGLE_MOTOR_ID = 7,
            FOLLOWER_ANGLE_MOTOR_ID = 8,
            ANGLE_ENCODER_ID = 1;
    private static final boolean
            MASTER_ANGLE_MOTOR_INVERTED = false,
            FOLLOWER_ANGLE_MOTOR_INVERTED = false;
    private static final double
            ANGLE_MOTOR_P = 8.5,
            ANGLE_MOTOR_I = 0,
            ANGLE_MOTOR_D = 0;
    static final PIDController ANGLE_PID_CONTROLLER = new PIDController(
            ANGLE_MOTOR_P, ANGLE_MOTOR_I, ANGLE_MOTOR_D
    );
    private static final CANSparkMax.IdleMode ANGLE_MOTOR_DEFAULT_NEUTRAL_MODE = CANSparkMax.IdleMode.kBrake;
    private static final int ANGLE_CURRENT_LIMIT = 70;
    // fully backwards: 102.480469
    static final double ANGLE_ENCODER_OFFSET = Conversions.degreesToRevolutions(135.791016 + 90 + 2 + 5);
    static final CANSparkMax
            MASTER_ANGLE_MOTOR = new CANSparkMax(MASTER_ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless),
            FOLLOWER_ANGLE_MOTOR = new CANSparkMax(FOLLOWER_ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    static final CANcoder ANGLE_ENCODER = new CANcoder(ANGLE_ENCODER_ID);

    private static final int
            MASTER_ELEVATOR_MOTOR_ID = 5,
            FOLLOWER_ELEVATOR_MOTOR_ID = 6,
            ELEVATOR_ENCODER_ID = 7;
    private static final boolean
            MASTER_ELEVATOR_MOTOR_INVERTED = false,
            FOLLOWER_ELEVATOR_MOTOR_INVERTED = true,
            ELEVATOR_ENCODER_PHASE = true;
    private static final double
            ELEVATOR_MOTOR_P = 6,
            ELEVATOR_MOTOR_I = 0,
            ELEVATOR_MOTOR_D = 0;
    static final PIDController ELEVATOR_PID_CONTROLLER = new PIDController(
            ELEVATOR_MOTOR_P, ELEVATOR_MOTOR_I, ELEVATOR_MOTOR_D
    );
    private static final CANSparkMax.IdleMode ELEVATOR_MOTOR_DEFAULT_NEUTRAL_MODE = CANSparkMax.IdleMode.kBrake;
    private static final double ELEVATOR_ENCODER_OFFSET = 0;
    private static final int ELEVATOR_CURRENT_LIMIT = 25;
    static final CANSparkMax
            MASTER_ELEVATOR_MOTOR = new CANSparkMax(MASTER_ELEVATOR_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless),
            FOLLOWER_ELEVATOR_MOTOR = new CANSparkMax(FOLLOWER_ELEVATOR_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    static final SRXMagEncoder ELEVATOR_ENCODER = new SRXMagEncoder(ELEVATOR_ENCODER_ID);

    private static final double
            ELEVATOR_MOTOR_KS = -0.00069393,
            ELEVATOR_MOTOR_KV = 1.6342,
            ELEVATOR_MOTOR_KA = 0.39459,
            ELEVATOR_MOTOR_KG = 0.097376;
    private static final ElevatorFeedforward ELEVATOR_MOTOR_FEEDFORWARD = new ElevatorFeedforward(
            ELEVATOR_MOTOR_KS, ELEVATOR_MOTOR_KG, ELEVATOR_MOTOR_KV, ELEVATOR_MOTOR_KA
    );
    private static final double
            CLOSED_ANGLE_MOTOR_KS = 0.58835,
            CLOSED_ANGLE_MOTOR_KV = 0.74627,
            CLOSED_ANGLE_MOTOR_KA = 0.37502,
            CLOSED_ANGLE_MOTOR_KG = 0.92056,
            FULLY_OPEN_ANGLE_MOTOR_KS = 2.4,
            FULLY_OPEN_ANGLE_MOTOR_KV = 2.2,
            FULLY_OPEN_ANGLE_MOTOR_KA = 0,
            FULLY_OPEN_ANGLE_MOTOR_KG = 2.4, // 1.63
            HALF_OPEN_ANGLE_MOTOR_KS = 2.1,
            HALF_OPEN_ANGLE_MOTOR_KV = 2.2,
            HALF_OPEN_ANGLE_MOTOR_KA = 0,
            HALF_OPEN_ANGLE_MOTOR_KG = 1.6; // 1.18
    private static final ArmFeedforward
            CLOSED_ANGLE_MOTOR_FEEDFORWARD = new ArmFeedforward(
                    CLOSED_ANGLE_MOTOR_KS, CLOSED_ANGLE_MOTOR_KG,
                    CLOSED_ANGLE_MOTOR_KV, CLOSED_ANGLE_MOTOR_KA
            ),
            FULLY_OPEN_ANGLE_MOTOR_FEEDFORWARD = new ArmFeedforward(
                    FULLY_OPEN_ANGLE_MOTOR_KS, FULLY_OPEN_ANGLE_MOTOR_KG,
                    FULLY_OPEN_ANGLE_MOTOR_KV, FULLY_OPEN_ANGLE_MOTOR_KA
            ),
            HALF_OPEN_ANGLE_MOTOR_FEEDFORWARD = new ArmFeedforward(
                    HALF_OPEN_ANGLE_MOTOR_KS, HALF_OPEN_ANGLE_MOTOR_KG,
                    HALF_OPEN_ANGLE_MOTOR_KV, HALF_OPEN_ANGLE_MOTOR_KA
            );
    private static final double
            FULLY_OPEN_REVOLUTIONS = 6.5,
            HALF_OPEN_REVOLUTIONS = 3.5;
    private static final HashMap<Double, ArmFeedforward> HEIGHT_TO_ANGLE_MOTOR_FEEDFORWARD_MAP = new HashMap<>();

    static StatusSignal<Double> ANGLE_POSITION_SIGNAL, ANGLE_VELOCITY_SIGNAL;

    static {
        HEIGHT_TO_ANGLE_MOTOR_FEEDFORWARD_MAP.put(0.0, CLOSED_ANGLE_MOTOR_FEEDFORWARD);
        HEIGHT_TO_ANGLE_MOTOR_FEEDFORWARD_MAP.put(FULLY_OPEN_REVOLUTIONS, FULLY_OPEN_ANGLE_MOTOR_FEEDFORWARD);
        HEIGHT_TO_ANGLE_MOTOR_FEEDFORWARD_MAP.put(HALF_OPEN_REVOLUTIONS, HALF_OPEN_ANGLE_MOTOR_FEEDFORWARD);
        HEIGHT_TO_ANGLE_MOTOR_FEEDFORWARD_MAP.put(0.5, CLOSED_ANGLE_MOTOR_FEEDFORWARD);
        ANGLE_PID_CONTROLLER.enableContinuousInput(-180, 180);

        if (!RobotConstants.IS_REPLAY) {
            configureEncoders();
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
        FOLLOWER_ANGLE_MOTOR.restoreFactoryDefaults();

        MASTER_ANGLE_MOTOR.enableVoltageCompensation(VOLTAGE_COMPENSATION_SATURATION);
        FOLLOWER_ANGLE_MOTOR.enableVoltageCompensation(VOLTAGE_COMPENSATION_SATURATION);

        MASTER_ANGLE_MOTOR.setInverted(MASTER_ANGLE_MOTOR_INVERTED);
        FOLLOWER_ANGLE_MOTOR.setInverted(FOLLOWER_ANGLE_MOTOR_INVERTED);

        MASTER_ANGLE_MOTOR.setIdleMode(ANGLE_MOTOR_DEFAULT_NEUTRAL_MODE);
        FOLLOWER_ANGLE_MOTOR.setIdleMode(ANGLE_MOTOR_DEFAULT_NEUTRAL_MODE);

        MASTER_ANGLE_MOTOR.setSmartCurrentLimit(ANGLE_CURRENT_LIMIT);
        FOLLOWER_ANGLE_MOTOR.setSmartCurrentLimit(ANGLE_CURRENT_LIMIT);

        MASTER_ANGLE_MOTOR.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0,255); // Applied output
        MASTER_ANGLE_MOTOR.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 255); // Motor movement
        FOLLOWER_ANGLE_MOTOR.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0,10000); // Applied output
        FOLLOWER_ANGLE_MOTOR.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 10000); // Motor movement
        for (CANSparkMax currentAngleMotor : List.of(MASTER_ANGLE_MOTOR, FOLLOWER_ANGLE_MOTOR)) {
            currentAngleMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 10000); // Motor position
            currentAngleMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 10000); // Analog sensor
            currentAngleMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus4, 10000); // Alternate encoder
            currentAngleMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus5, 10000); // Duty cycle position
            currentAngleMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus6, 10000); // Duty cycle velocity
        }

//        FOLLOWER_ANGLE_MOTOR.follow(MASTER_ANGLE_MOTOR);

        new Notifier(() -> {MASTER_ANGLE_MOTOR.burnFlash(); FOLLOWER_ANGLE_MOTOR.burnFlash();}).startSingle(0.03);
    }

    private static void configureElevatorMotor() {
        MASTER_ELEVATOR_MOTOR.restoreFactoryDefaults();
        FOLLOWER_ELEVATOR_MOTOR.restoreFactoryDefaults();

        MASTER_ELEVATOR_MOTOR.enableVoltageCompensation(VOLTAGE_COMPENSATION_SATURATION);
        FOLLOWER_ELEVATOR_MOTOR.enableVoltageCompensation(VOLTAGE_COMPENSATION_SATURATION);

        MASTER_ELEVATOR_MOTOR.setInverted(MASTER_ELEVATOR_MOTOR_INVERTED);
        FOLLOWER_ELEVATOR_MOTOR.setInverted(FOLLOWER_ELEVATOR_MOTOR_INVERTED);

        MASTER_ELEVATOR_MOTOR.setIdleMode(ELEVATOR_MOTOR_DEFAULT_NEUTRAL_MODE);
        FOLLOWER_ELEVATOR_MOTOR.setIdleMode(ELEVATOR_MOTOR_DEFAULT_NEUTRAL_MODE);

        MASTER_ELEVATOR_MOTOR.setSmartCurrentLimit(ELEVATOR_CURRENT_LIMIT);
        FOLLOWER_ELEVATOR_MOTOR.setSmartCurrentLimit(ELEVATOR_CURRENT_LIMIT);

        MASTER_ELEVATOR_MOTOR.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 255); // Applied output
        MASTER_ELEVATOR_MOTOR.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 255); // Motor movement
        FOLLOWER_ELEVATOR_MOTOR.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100000); // Applied output
        FOLLOWER_ELEVATOR_MOTOR.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 100000); // Motor movement
        for (CANSparkMax currentElevatorMotor : List.of(MASTER_ELEVATOR_MOTOR, FOLLOWER_ELEVATOR_MOTOR)) {
            currentElevatorMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 10000); // Motor position
            currentElevatorMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 10000); // Analog sensor
            currentElevatorMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus4, 10000); // Alternate encoder
            currentElevatorMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus5, 10000); // Duty cycle position
            currentElevatorMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus6, 10000); // Duty cycle velocity
        }

//        FOLLOWER_ELEVATOR_MOTOR.follow(MASTER_ELEVATOR_MOTOR, FOLLOWER_ELEVATOR_MOTOR_INVERTS_MASTER);

        new Notifier(() -> {MASTER_ELEVATOR_MOTOR.burnFlash(); FOLLOWER_ELEVATOR_MOTOR.burnFlash();}).startSingle(0.03);
    }

    private static void configureEncoders() {
        ELEVATOR_ENCODER.configFactoryDefault();
        ELEVATOR_ENCODER.setSensorPhase(ELEVATOR_ENCODER_PHASE);
        ELEVATOR_ENCODER.setPositionScale(1);
        ELEVATOR_ENCODER.setVelocityScale(1);
        ELEVATOR_ENCODER.setOffset(ELEVATOR_ENCODER_OFFSET);

        final CANcoderConfiguration configuration = new CANcoderConfiguration();

        configuration.MagnetSensor.MagnetOffset = ANGLE_ENCODER_OFFSET;
        configuration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

        ANGLE_ENCODER.getConfigurator().apply(configuration);

        ANGLE_VELOCITY_SIGNAL = ANGLE_ENCODER.getVelocity();
        ANGLE_POSITION_SIGNAL = ANGLE_ENCODER.getAbsolutePosition();
        ANGLE_VELOCITY_SIGNAL.setUpdateFrequency(50);
        ANGLE_POSITION_SIGNAL.setUpdateFrequency(100);
    }
}
