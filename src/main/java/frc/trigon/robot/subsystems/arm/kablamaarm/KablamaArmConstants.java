package frc.trigon.robot.subsystems.arm.kablamaarm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.arm.ArmConstants;
import frc.trigon.robot.subsystems.collector.kablamacollector.KablamaCollectorConstants;
import frc.trigon.robot.utilities.SRXMagEncoder;

import java.util.HashMap;
import java.util.List;

public class KablamaArmConstants extends ArmConstants {
    private static final double VOLTAGE_COMPENSATION_SATURATION = 12;

    private static final int
            MASTER_ANGLE_MOTOR_ID = 7,
            FOLLOWER_ANGLE_MOTOR_ID = 8;
    private static final boolean
            MASTER_ANGLE_MOTOR_INVERTED = false,
            FOLLOWER_ANGLE_MOTOR_INVERTED = false;
    private static final double
            ANGLE_MOTOR_P = 8,
            ANGLE_MOTOR_I = 1.5,
            ANGLE_MOTOR_D = 0;
    static final PIDController ANGLE_PID_CONTROLLER = new PIDController(
            ANGLE_MOTOR_P, ANGLE_MOTOR_I, ANGLE_MOTOR_D
    );
    private static final CANSparkMax.IdleMode ANGLE_MOTOR_DEFAULT_NEUTRAL_MODE = CANSparkMax.IdleMode.kBrake;
    private static final int ANGLE_CURRENT_LIMIT = 30;
    static final double ANGLE_ENCODER_OFFSET = -19.335938;
    static final CANSparkMax
            MASTER_ANGLE_MOTOR = new CANSparkMax(MASTER_ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless),
            FOLLOWER_ANGLE_MOTOR = new CANSparkMax(FOLLOWER_ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    static final SRXMagEncoder ANGLE_ENCODER = new SRXMagEncoder(KablamaCollectorConstants.MOTOR);

    private static final int
            MASTER_ELEVATOR_MOTOR_ID = 5,
            FOLLOWER_ELEVATOR_MOTOR_ID = 6,
            ELEVATOR_ENCODER_ID = 7;
    private static final boolean
            MASTER_ELEVATOR_MOTOR_INVERTED = false,
            FOLLOWER_ELEVATOR_MOTOR_INVERTED = true,
            ELEVATOR_ENCODER_PHASE = true;
    private static final double
            ELEVATOR_MOTOR_P = 2,
            ELEVATOR_MOTOR_I = 0,
            ELEVATOR_MOTOR_D = 0;
    static final PIDController ELEVATOR_PID_CONTROLLER = new PIDController(
            ELEVATOR_MOTOR_P, ELEVATOR_MOTOR_I, ELEVATOR_MOTOR_D
    );
    private static final CANSparkMax.IdleMode ELEVATOR_MOTOR_DEFAULT_NEUTRAL_MODE = CANSparkMax.IdleMode.kBrake;
    private static final double ELEVATOR_ENCODER_OFFSET = 0.038330;
    private static final int ELEVATOR_CURRENT_LIMIT = 30;
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
            CLOSED_ANGLE_MOTOR_KS = 0.12854,
            CLOSED_ANGLE_MOTOR_KV = 0.92886,
            CLOSED_ANGLE_MOTOR_KA = 0.05869,
            CLOSED_ANGLE_MOTOR_KG = 0.47416,
            FULLY_OPEN_ANGLE_MOTOR_KS = 0.3,
            FULLY_OPEN_ANGLE_MOTOR_KV = 2,
            FULLY_OPEN_ANGLE_MOTOR_KA = 0,
            FULLY_OPEN_ANGLE_MOTOR_KG = 2,
            HALF_OPEN_ANGLE_MOTOR_KS = 0.18,
            HALF_OPEN_ANGLE_MOTOR_KV = 1.2,
            HALF_OPEN_ANGLE_MOTOR_KA = 0,
            HALF_OPEN_ANGLE_MOTOR_KG = 0.7;
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
    private static final HashMap<Double, ArmFeedforward> HEIGHT_TO_ANGLE_MOTOR_FEEDFORWARD_MAP = new HashMap<>();

    private static final int
            ANGLE_LIMIT_SWITCH_CHANNEL = 1,
            FORWARD_ELEVATOR_LIMIT_SWITCH_CHANNEL = 2,
            REVERSE_ELEVATOR_LIMIT_SWITCH_CHANNEL = 3;
    //    private static final DigitalInput
//            ANGLE_LIMIT_SWITCH = new DigitalInput(ANGLE_LIMIT_SWITCH_CHANNEL),
//            FORWARD_ELEVATOR_LIMIT_SWITCH = new DigitalInput(FORWARD_ELEVATOR_LIMIT_SWITCH_CHANNEL),
//            REVERSE_ELEVATOR_LIMIT_SWITCH = new DigitalInput(REVERSE_ELEVATOR_LIMIT_SWITCH_CHANNEL);
    private static final double
            ANGLE_LIMIT_SWITCH_PRESSED_POSITION = 0,
            FORWARD_ELEVATOR_LIMIT_SWITCH_PRESSED_POSITION = 0,
            REVERSE_ELEVATOR_LIMIT_SWITCH_PRESSED_POSITION = 0;
    private static final double LIMIT_SWITCH_WATCHER_TIME_THRESHOLD = 0.5990;
    private static final float ANGLE_MOTOR_FORWARD_SOFT_LIMIT = 100;


    static {
        HEIGHT_TO_ANGLE_MOTOR_FEEDFORWARD_MAP.put(0.0, CLOSED_ANGLE_MOTOR_FEEDFORWARD);
        HEIGHT_TO_ANGLE_MOTOR_FEEDFORWARD_MAP.put(6.683350, FULLY_OPEN_ANGLE_MOTOR_FEEDFORWARD);
        HEIGHT_TO_ANGLE_MOTOR_FEEDFORWARD_MAP.put(3.5, HALF_OPEN_ANGLE_MOTOR_FEEDFORWARD);
        ANGLE_PID_CONTROLLER.enableContinuousInput(-180, 180);

        if (!RobotConstants.IS_REPLAY) {
            configureEncoders();
            configureElevatorMotor();
            configureAngleMotor();
            configureLimitSwitchWatchers();
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

//        MASTER_ANGLE_MOTOR.getPIDController().setP(ANGLE_MOTOR_P);
//        MASTER_ANGLE_MOTOR.getPIDController().setI(ANGLE_MOTOR_I);
//        MASTER_ANGLE_MOTOR.getPIDController().setD(ANGLE_MOTOR_D);
//        MASTER_ANGLE_MOTOR.getPIDController().setPositionPIDWrappingEnabled(true);
//
//        ANGLE_ENCODER.setPositionConversionFactor(360);
//        ANGLE_ENCODER.setVelocityConversionFactor(360);
//        MASTER_ANGLE_MOTOR.getPIDController().setFeedbackDevice(ANGLE_ENCODER);

        MASTER_ANGLE_MOTOR.setIdleMode(ANGLE_MOTOR_DEFAULT_NEUTRAL_MODE);
        FOLLOWER_ANGLE_MOTOR.setIdleMode(ANGLE_MOTOR_DEFAULT_NEUTRAL_MODE);

        MASTER_ANGLE_MOTOR.setSmartCurrentLimit(ANGLE_CURRENT_LIMIT);
        FOLLOWER_ANGLE_MOTOR.setSmartCurrentLimit(ANGLE_CURRENT_LIMIT);

        MASTER_ANGLE_MOTOR.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0,20); // Applied output
        MASTER_ANGLE_MOTOR.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20); // Motor movement
        FOLLOWER_ANGLE_MOTOR.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0,150); // Applied output
        FOLLOWER_ANGLE_MOTOR.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 150); // Motor movement
        for (CANSparkMax currentAngleMotor : List.of(MASTER_ANGLE_MOTOR, FOLLOWER_ANGLE_MOTOR)) {
            currentAngleMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 1000); // Motor position
            currentAngleMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 1000); // Analog sensor
            currentAngleMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus4, 1000); // Alternate encoder
            currentAngleMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus5, 1000); // Duty cycle position
            currentAngleMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus6, 1000); // Duty cycle velocity
        }

//        MASTER_ANGLE_MOTOR.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) ANGLE_LIMIT_SWITCH_PRESSED_POSITION);
//        MASTER_ANGLE_MOTOR.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ANGLE_MOTOR_FORWARD_SOFT_LIMIT);

        FOLLOWER_ANGLE_MOTOR.follow(MASTER_ANGLE_MOTOR);

        MASTER_ANGLE_MOTOR.burnFlash();
        FOLLOWER_ANGLE_MOTOR.burnFlash();
    }

    private static void configureElevatorMotor() {
        MASTER_ELEVATOR_MOTOR.restoreFactoryDefaults();
        FOLLOWER_ELEVATOR_MOTOR.restoreFactoryDefaults();

        MASTER_ELEVATOR_MOTOR.enableVoltageCompensation(VOLTAGE_COMPENSATION_SATURATION);
        FOLLOWER_ELEVATOR_MOTOR.enableVoltageCompensation(VOLTAGE_COMPENSATION_SATURATION);

        MASTER_ELEVATOR_MOTOR.setInverted(MASTER_ELEVATOR_MOTOR_INVERTED);

//        MASTER_ELEVATOR_MOTOR.getPIDController().setP(ELEVATOR_MOTOR_P);
//        MASTER_ELEVATOR_MOTOR.getPIDController().setI(ELEVATOR_MOTOR_I);
//        MASTER_ELEVATOR_MOTOR.getPIDController().setD(ELEVATOR_MOTOR_D);
//        MASTER_ELEVATOR_MOTOR.getPIDController().setPositionPIDWrappingEnabled(true);
//
//        MASTER_ELEVATOR_MOTOR.getPIDController().setFeedbackDevice(ELEVATOR_ENCODER);

        MASTER_ELEVATOR_MOTOR.setIdleMode(ELEVATOR_MOTOR_DEFAULT_NEUTRAL_MODE);
        FOLLOWER_ELEVATOR_MOTOR.setIdleMode(ELEVATOR_MOTOR_DEFAULT_NEUTRAL_MODE);

        MASTER_ELEVATOR_MOTOR.setSmartCurrentLimit(ELEVATOR_CURRENT_LIMIT);
        FOLLOWER_ELEVATOR_MOTOR.setSmartCurrentLimit(ELEVATOR_CURRENT_LIMIT);

        MASTER_ELEVATOR_MOTOR.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 20); // Applied output
        MASTER_ELEVATOR_MOTOR.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20); // Motor movement
        FOLLOWER_ELEVATOR_MOTOR.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 150); // Applied output
        FOLLOWER_ELEVATOR_MOTOR.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 150); // Motor movement
        for (CANSparkMax currentElevatorMotor : List.of(MASTER_ELEVATOR_MOTOR, FOLLOWER_ELEVATOR_MOTOR)) {
            currentElevatorMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 1000); // Motor position
            currentElevatorMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 1000); // Analog sensor
            currentElevatorMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus4, 1000); // Alternate encoder
            currentElevatorMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus5, 1000); // Duty cycle position
            currentElevatorMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus6, 1000); // Duty cycle velocity
        }

//        MASTER_ELEVATOR_MOTOR.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) REVERSE_ELEVATOR_LIMIT_SWITCH_PRESSED_POSITION);
//        MASTER_ELEVATOR_MOTOR.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) FORWARD_ELEVATOR_LIMIT_SWITCH_PRESSED_POSITION);

        FOLLOWER_ELEVATOR_MOTOR.follow(MASTER_ELEVATOR_MOTOR, FOLLOWER_ELEVATOR_MOTOR_INVERTED);

        MASTER_ELEVATOR_MOTOR.burnFlash();
        FOLLOWER_ELEVATOR_MOTOR.burnFlash();
    }

    private static void configureEncoders() {
        ELEVATOR_ENCODER.configFactoryDefault();
        ELEVATOR_ENCODER.setSensorPhase(ELEVATOR_ENCODER_PHASE);
        ELEVATOR_ENCODER.setPositionScale(1);
        ELEVATOR_ENCODER.setVelocityScale(1);
        ELEVATOR_ENCODER.setOffset(ELEVATOR_ENCODER_OFFSET);

        ANGLE_ENCODER.setVelocityScale(360);
        ANGLE_ENCODER.setPositionScale(360);
        ANGLE_ENCODER.setOffset(ANGLE_ENCODER_OFFSET);
        ANGLE_ENCODER.enableContinuousPosition(-180, 180);
    }

    private static void configureLimitSwitchWatchers() {
//        new LimitSwitchWatcher(
////                ANGLE_LIMIT_SWITCH::get,
//                () -> false,
//                LIMIT_SWITCH_WATCHER_TIME_THRESHOLD,
//                true,
//                () -> ANGLE_ENCODER.setSelectedSensorPosition(ANGLE_LIMIT_SWITCH_PRESSED_POSITION)
//        );
//        new LimitSwitchWatcher(
////                FORWARD_ELEVATOR_LIMIT_SWITCH::get,
//                () -> false,
//                LIMIT_SWITCH_WATCHER_TIME_THRESHOLD,
//                true,
//                () -> {
//                    if (DriverStation.isEnabled())
//                        ELEVATOR_ENCODER.setSelectedSensorPosition(FORWARD_ELEVATOR_LIMIT_SWITCH_PRESSED_POSITION);
//                }
//        );
//        new LimitSwitchWatcher(
////                REVERSE_ELEVATOR_LIMIT_SWITCH::get,
//                () -> false,
//                LIMIT_SWITCH_WATCHER_TIME_THRESHOLD,
//                true,
//                () -> {
//                    if (DriverStation.isEnabled())
//                        ELEVATOR_ENCODER.setSelectedSensorPosition(REVERSE_ELEVATOR_LIMIT_SWITCH_PRESSED_POSITION);
//                }
//        );
    }
}
