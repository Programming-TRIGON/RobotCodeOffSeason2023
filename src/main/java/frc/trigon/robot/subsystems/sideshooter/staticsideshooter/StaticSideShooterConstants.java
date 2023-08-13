package frc.trigon.robot.subsystems.sideshooter.staticsideshooter;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import frc.trigon.robot.subsystems.sideshooter.SideShooterConstants;

public class StaticSideShooterConstants extends SideShooterConstants {
    static final double VOLTAGE_COMPENSATION_SATURATION = 12;

    private static final boolean
            SHOOTING_MOTOR_INVERTED = false,
            ANGLE_MOTOR_INVERTED = false;
    private static final int
            SHOOTING_MOTOR_ID = 1,
            ANGLE_MOTOR_ID = 1;
    private static final NeutralMode DEFAULT_SHOOTING_MOTOR_NEUTRAL_MODE = NeutralMode.Brake;
    private static final CANSparkMax.IdleMode DEFAULT_ANGLE_MOTOR_IDLE_MODE = CANSparkMax.IdleMode.kBrake;
    private static final double
            ANGLE_MOTOR_P = 1,
            ANGLE_MOTOR_I = 0,
            ANGLE_MOTOR_D = 0;
    private static final double ANGLE_ENCODER_OFFSET = 0;
    private static final int ANGLE_MOTOR_CURRENT_LIMIT = 30;

    static final WPI_TalonSRX SHOOTING_MOTOR = new WPI_TalonSRX(SHOOTING_MOTOR_ID);
    static final CANSparkMax ANGLE_MOTOR = new CANSparkMax(ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    static final SparkMaxAbsoluteEncoder ANGLE_ENCODER = ANGLE_MOTOR.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

    private static final double
            ANGLE_MOTOR_KS = 0,
            ANGLE_MOTOR_KV = 0,
            ANGLE_MOTOR_KA = 0,
            ANGLE_MOTOR_KG = 0;
    private static final ArmFeedforward ANGLE_MOTOR_FEEDFORWARD = new ArmFeedforward(
            ANGLE_MOTOR_KS, ANGLE_MOTOR_KG, ANGLE_MOTOR_KV, ANGLE_MOTOR_KA
    );

    static {
        configureShootingMotor();
        configureAngleMotor();
    }

    @Override
    protected ArmFeedforward getAngleMotorFeedforward() {
        return ANGLE_MOTOR_FEEDFORWARD;
    }

    private static void configureShootingMotor() {
        SHOOTING_MOTOR.configFactoryDefault();

        SHOOTING_MOTOR.configVoltageCompSaturation(VOLTAGE_COMPENSATION_SATURATION);
        SHOOTING_MOTOR.enableVoltageCompensation(true);

        SHOOTING_MOTOR.setInverted(SHOOTING_MOTOR_INVERTED);
        SHOOTING_MOTOR.setNeutralMode(DEFAULT_SHOOTING_MOTOR_NEUTRAL_MODE);
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
