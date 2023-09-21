package frc.trigon.robot.subsystems.roller.kablamaroller;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;

public class KablamaRollerConstants {
    static final double VOLTAGE_COMPENSATION_SATURATION = 12;

    private static final boolean
            ANGLE_MOTOR_INVERTED = false,
            COLLECTION_MOTOR_INVERTED = false;
    private static final int
            ANGLE_MOTOR_ID = 6,
            COLLECTION_MOTOR_ID = 10;
    private static final CANSparkMax.IdleMode DEFAULT_COLLECTION_MOTOR_IDLE_MODE = CANSparkMax.IdleMode.kCoast;
    private static final NeutralMode DEFAULT_ANGLE_MOTOR_IDLE_MODE = NeutralMode.Brake;
    private static final int
            ANGLE_MOTOR_CURRENT_LIMIT = 30,
            COLLECTION_MOTOR_CURRENT_LIMIT = 30;

    public static final WPI_TalonSRX ANGLE_MOTOR = new WPI_TalonSRX(ANGLE_MOTOR_ID);
    static final CANSparkMax COLLECTION_MOTOR = new CANSparkMax(COLLECTION_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

    private static final int
            FORWARD_ANGLE_LIMIT_SWITCH_CHANNEL = 7,
            REVERSE_ANGLE_LIMIT_SWITCH_CHANNEL = 5;
    static final DigitalInput
//        FORWARD_ANGLE_LIMIT_SWITCH = new DigitalInput(FORWARD_ANGLE_LIMIT_SWITCH_CHANNEL);
        REVERSE_ANGLE_LIMIT_SWITCH = new DigitalInput(REVERSE_ANGLE_LIMIT_SWITCH_CHANNEL);

    static {
        configureAngleMotor();
        configureCollectionMotor();
    }

    private static void configureAngleMotor() {
        ANGLE_MOTOR.configFactoryDefault();

        ANGLE_MOTOR.configVoltageCompSaturation(VOLTAGE_COMPENSATION_SATURATION);
        ANGLE_MOTOR.enableVoltageCompensation(true);
        ANGLE_MOTOR.setInverted(ANGLE_MOTOR_INVERTED);
        ANGLE_MOTOR.setNeutralMode(DEFAULT_ANGLE_MOTOR_IDLE_MODE);
        ANGLE_MOTOR.configPeakCurrentLimit(ANGLE_MOTOR_CURRENT_LIMIT);
        ANGLE_MOTOR.enableCurrentLimit(true);
    }

    private static void configureCollectionMotor() {
        COLLECTION_MOTOR.restoreFactoryDefaults();

        COLLECTION_MOTOR.enableVoltageCompensation(VOLTAGE_COMPENSATION_SATURATION);
        COLLECTION_MOTOR.setInverted(COLLECTION_MOTOR_INVERTED);
        COLLECTION_MOTOR.setIdleMode(DEFAULT_COLLECTION_MOTOR_IDLE_MODE);
        COLLECTION_MOTOR.setSmartCurrentLimit(COLLECTION_MOTOR_CURRENT_LIMIT);

        COLLECTION_MOTOR.burnFlash();
    }
}
