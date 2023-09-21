package frc.trigon.robot.subsystems.sideshooter.kablamasideshooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.roller.kablamaroller.KablamaRollerConstants;
import frc.trigon.robot.subsystems.sideshooter.SideShooterConstants;
import frc.trigon.robot.utilities.SRXMagEncoder;

public class KablamaSideShooterConstants extends SideShooterConstants {
    static final double VOLTAGE_COMPENSATION_SATURATION = 12;
    static final boolean SHOOTING_MOTOR_FOC = false;

    private static final boolean ANGLE_MOTOR_INVERTED = false;
    private static final InvertedValue SHOOTING_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive;
    private static final int
            SHOOTING_MOTOR_ID = 0,
            ANGLE_MOTOR_ID = 9;
    private static final NeutralModeValue DEFAULT_SHOOTING_MOTOR_NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    private static final CANSparkMax.IdleMode DEFAULT_ANGLE_MOTOR_IDLE_MODE = CANSparkMax.IdleMode.kBrake;
    private static final double
            ANGLE_MOTOR_P = 8,
            ANGLE_MOTOR_I = 0,
            ANGLE_MOTOR_D = 0;
    static final PIDController ANGLE_PID_CONTROLLER = new PIDController(
            ANGLE_MOTOR_P, ANGLE_MOTOR_I, ANGLE_MOTOR_D
    );
    private static final int ANGLE_MOTOR_CURRENT_LIMIT = 30;
    static final double ANGLE_ENCODER_OFFSET = 87.187500 - 90;
    static final boolean ANGLE_ENCODER_PHASE = true;

    static final TalonFX SHOOTING_MOTOR = new TalonFX(SHOOTING_MOTOR_ID);
    static final CANSparkMax ANGLE_MOTOR = new CANSparkMax(ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    static final SRXMagEncoder ANGLE_ENCODER = new SRXMagEncoder(KablamaRollerConstants.ANGLE_MOTOR);

    static StatusSignal<Double> STATOR_CURRENT_SIGNAL, MOTOR_OUTPUT_PERCENT_SIGNAL;

    private static final double
            ANGLE_MOTOR_KS = 0,
            ANGLE_MOTOR_KV = 0,
            ANGLE_MOTOR_KA = 5.7625,
            ANGLE_MOTOR_KG = 0;
    private static final ArmFeedforward ANGLE_MOTOR_FEEDFORWARD = new ArmFeedforward(
            ANGLE_MOTOR_KS, ANGLE_MOTOR_KG, ANGLE_MOTOR_KV, ANGLE_MOTOR_KA
    );

    private static final int
            FORWARD_ANGLE_MOTOR_LIMIT_SWITCH_CHANNEL = 2,
            REVERSE_ANGLE_MOTOR_LIMIT_SWITCH_CHANNEL = 3;
    //    private static final DigitalInput
//            FORWARD_ANGLE_MOTOR_LIMIT_SWITCH = new DigitalInput(FORWARD_ANGLE_MOTOR_LIMIT_SWITCH_CHANNEL),
//            REVERSE_ANGLE_MOTOR_LIMIT_SWITCH = new DigitalInput(REVERSE_ANGLE_MOTOR_LIMIT_SWITCH_CHANNEL);
    private static final double
            FORWARD_ANGLE_MOTOR_LIMIT_SWITCH_PRESSED_POSITION = 0,
            REVERSE_ANGLE_MOTOR_LIMIT_SWITCH_PRESSED_POSITION = 0;
    private static final double LIMIT_SWITCH_WATCHER_TIME_THRESHOLD = 0.5990;

    static {
        ANGLE_PID_CONTROLLER.enableContinuousInput(-180, 180);

        if (!RobotConstants.IS_REPLAY) {
            configureAngleEncoder();
            configureShootingMotor();
            configureAngleMotor();
            configureLimitSwitchWatchers();
        }
    }

    @Override
    protected ArmFeedforward getAngleMotorFeedforward() {
        return ANGLE_MOTOR_FEEDFORWARD;
    }

    private static void configureAngleEncoder() {
        ANGLE_ENCODER.setPositionScale(360);
        ANGLE_ENCODER.setVelocityScale(360);

        ANGLE_ENCODER.setSensorPhase(ANGLE_ENCODER_PHASE);
        ANGLE_ENCODER.setOffset(ANGLE_ENCODER_OFFSET);
        ANGLE_ENCODER.enableContinuousPosition(-180, 180);
    }

    private static void configureShootingMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.MotorOutput.Inverted = SHOOTING_MOTOR_INVERTED_VALUE;
        config.MotorOutput.NeutralMode = DEFAULT_SHOOTING_MOTOR_NEUTRAL_MODE_VALUE;
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0;

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

//        ANGLE_MOTOR.getPIDController().setP(ANGLE_MOTOR_P);
//        ANGLE_MOTOR.getPIDController().setI(ANGLE_MOTOR_I);
//        ANGLE_MOTOR.getPIDController().setD(ANGLE_MOTOR_D);
//        ANGLE_MOTOR.getPIDController().setPositionPIDWrappingEnabled(true);
//
//        ANGLE_ENCODER.setPositionConversionFactor(360);
//        ANGLE_ENCODER.setVelocityConversionFactor(360);
//        ANGLE_MOTOR.getPIDController().setFeedbackDevice(ANGLE_ENCODER);

        ANGLE_MOTOR.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) FORWARD_ANGLE_MOTOR_LIMIT_SWITCH_PRESSED_POSITION);
        ANGLE_MOTOR.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) REVERSE_ANGLE_MOTOR_LIMIT_SWITCH_PRESSED_POSITION);

        ANGLE_MOTOR.burnFlash();
    }

    private static void configureLimitSwitchWatchers() {
//        new LimitSwitchWatcher(
////                FORWARD_ANGLE_MOTOR_LIMIT_SWITCH::get,
//                () -> false,
//                LIMIT_SWITCH_WATCHER_TIME_THRESHOLD,
//                true,
//                () -> {
//                    if (DriverStation.isEnabled()){}
//                        ANGLE_ENCODER.setSelectedSensorPosition(FORWARD_ANGLE_MOTOR_LIMIT_SWITCH_PRESSED_POSITION);
//                }
//        );
//        new LimitSwitchWatcher(
////                REVERSE_ANGLE_MOTOR_LIMIT_SWITCH::get,
//                () -> false,
//                LIMIT_SWITCH_WATCHER_TIME_THRESHOLD,
//                true,
//                () -> {
//                    if (DriverStation.isEnabled()){}
//                        ANGLE_ENCODER.setSelectedSensorPosition(REVERSE_ANGLE_MOTOR_LIMIT_SWITCH_PRESSED_POSITION);
//                }
//        );
    }
}
