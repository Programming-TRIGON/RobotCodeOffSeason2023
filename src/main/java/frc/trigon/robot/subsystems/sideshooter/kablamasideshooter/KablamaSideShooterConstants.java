package frc.trigon.robot.subsystems.sideshooter.kablamasideshooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Notifier;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.sideshooter.SideShooterConstants;
import frc.trigon.robot.utilities.Conversions;

public class KablamaSideShooterConstants extends SideShooterConstants {
    static final double VOLTAGE_COMPENSATION_SATURATION = 12;
    static final boolean SHOOTING_MOTOR_FOC = true;

    private static final boolean ANGLE_MOTOR_INVERTED = false;
    private static final InvertedValue SHOOTING_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive;
    private static final int
            SHOOTING_MOTOR_ID = 0,
            ANGLE_MOTOR_ID = 9,
            ANGLE_ENCODER_ID = 0;
    private static final NeutralModeValue DEFAULT_SHOOTING_MOTOR_NEUTRAL_MODE_VALUE = NeutralModeValue.Coast;
    private static final CANSparkMax.IdleMode DEFAULT_ANGLE_MOTOR_IDLE_MODE = CANSparkMax.IdleMode.kBrake;
    private static final double
            ANGLE_MOTOR_P = 2,
            ANGLE_MOTOR_I = 0,
            ANGLE_MOTOR_D = 0;
    static final PIDController ANGLE_PID_CONTROLLER = new PIDController(
            ANGLE_MOTOR_P, ANGLE_MOTOR_I, ANGLE_MOTOR_D
    );
    private static final int ANGLE_MOTOR_CURRENT_LIMIT = 30;
    static final double ANGLE_ENCODER_OFFSET = Conversions.degreesToRevolutions(-177.011719 + 90);
    static final SensorDirectionValue ANGLE_ENCODER_SENSOR_DIRECTION = SensorDirectionValue.Clockwise_Positive;

    static final TalonFX SHOOTING_MOTOR = new TalonFX(SHOOTING_MOTOR_ID);
    static final CANSparkMax ANGLE_MOTOR = new CANSparkMax(ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    static final CANcoder ANGLE_ENCODER = new CANcoder(ANGLE_ENCODER_ID);

    static StatusSignal<Double> SHOOTING_STATOR_CURRENT_SIGNAL, SHOOTING_MOTOR_OUTPUT_PERCENT_SIGNAL, ANGLE_POSITION_SIGNAL, ANGLE_VELOCITY_SIGNAL;

    private static final double
            ANGLE_MOTOR_KS = 0.08,
            ANGLE_MOTOR_KV = 3.339,
            ANGLE_MOTOR_KA = 1.2795,
            ANGLE_MOTOR_KG = 0.6421;
    private static final ArmFeedforward ANGLE_MOTOR_FEEDFORWARD = new ArmFeedforward(
            ANGLE_MOTOR_KS, ANGLE_MOTOR_KG, ANGLE_MOTOR_KV, ANGLE_MOTOR_KA
    );

    static {
        ANGLE_PID_CONTROLLER.enableContinuousInput(-180, 180);

        if (!RobotConstants.IS_REPLAY) {
            configureAngleEncoder();
            configureShootingMotor();
            configureAngleMotor();
        }
    }

    @Override
    protected ArmFeedforward getAngleMotorFeedforward() {
        return ANGLE_MOTOR_FEEDFORWARD;
    }

    private static void configureAngleEncoder() {
        final CANcoderConfiguration configuration = new CANcoderConfiguration();

        configuration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        configuration.MagnetSensor.SensorDirection = ANGLE_ENCODER_SENSOR_DIRECTION;
        configuration.MagnetSensor.MagnetOffset = ANGLE_ENCODER_OFFSET;

        ANGLE_ENCODER.getConfigurator().apply(configuration);

        ANGLE_VELOCITY_SIGNAL = ANGLE_ENCODER.getVelocity();
        ANGLE_POSITION_SIGNAL = ANGLE_ENCODER.getAbsolutePosition();
        ANGLE_VELOCITY_SIGNAL.setUpdateFrequency(50);
        ANGLE_POSITION_SIGNAL.setUpdateFrequency(100);
        // TODO: status signals
    }

    private static void configureShootingMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.Audio.BeepOnBoot = false;
        config.MotorOutput.Inverted = SHOOTING_MOTOR_INVERTED_VALUE;
        config.MotorOutput.NeutralMode = DEFAULT_SHOOTING_MOTOR_NEUTRAL_MODE_VALUE;
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0;

        SHOOTING_MOTOR.getConfigurator().apply(config);

        SHOOTING_STATOR_CURRENT_SIGNAL = SHOOTING_MOTOR.getStatorCurrent();
        SHOOTING_MOTOR_OUTPUT_PERCENT_SIGNAL = SHOOTING_MOTOR.getDutyCycle();
        SHOOTING_STATOR_CURRENT_SIGNAL.setUpdateFrequency(10);
        SHOOTING_MOTOR_OUTPUT_PERCENT_SIGNAL.setUpdateFrequency(10);
        // TODO: status signals
    }

    private static void configureAngleMotor() {
        ANGLE_MOTOR.restoreFactoryDefaults();

        ANGLE_MOTOR.enableVoltageCompensation(VOLTAGE_COMPENSATION_SATURATION);
        ANGLE_MOTOR.setInverted(ANGLE_MOTOR_INVERTED);
        ANGLE_MOTOR.setIdleMode(DEFAULT_ANGLE_MOTOR_IDLE_MODE);
//        ANGLE_MOTOR.setSmartCurrentLimit(ANGLE_MOTOR_CURRENT_LIMIT);

        ANGLE_MOTOR.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 255); // Applied output
        ANGLE_MOTOR.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 255); // Motor movement
        ANGLE_MOTOR.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 10000); // Motor position
        ANGLE_MOTOR.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 10000); // Analog sensor
        ANGLE_MOTOR.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus4, 10000); // Alternate encoder
        ANGLE_MOTOR.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus5, 10000); // Duty cycle position
        ANGLE_MOTOR.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus6, 10000); // Duty cycle velocity

        new Notifier(ANGLE_MOTOR::burnFlash).startSingle(0.03);
    }
}
