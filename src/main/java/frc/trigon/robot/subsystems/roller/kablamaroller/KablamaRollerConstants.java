package frc.trigon.robot.subsystems.roller.kablamaroller;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Notifier;

public class KablamaRollerConstants {
    static final double VOLTAGE_COMPENSATION_SATURATION = 12;

    private static final boolean
            ANGLE_MOTOR_INVERTED = true,
            COLLECTION_MOTOR_INVERTED = false;
    private static final int
            ANGLE_MOTOR_ID = 6,
            COLLECTION_MOTOR_ID = 10;
    private static final CANSparkMax.IdleMode DEFAULT_COLLECTION_MOTOR_IDLE_MODE = CANSparkMax.IdleMode.kCoast;
    private static final NeutralMode DEFAULT_ANGLE_MOTOR_NEUTRAL_MODE = NeutralMode.Brake;

    static final WPI_TalonSRX ANGLE_MOTOR = new WPI_TalonSRX(ANGLE_MOTOR_ID);
    static final CANSparkMax COLLECTION_MOTOR = new CANSparkMax(COLLECTION_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

    private static final int
            FORWARD_ANGLE_LIMIT_SWITCH_CHANNEL = 2,
            REVERSE_ANGLE_LIMIT_SWITCH_CHANNEL = 0;
    static final DigitalInput
        FORWARD_ANGLE_LIMIT_SWITCH = new DigitalInput(FORWARD_ANGLE_LIMIT_SWITCH_CHANNEL),
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
        ANGLE_MOTOR.setNeutralMode(DEFAULT_ANGLE_MOTOR_NEUTRAL_MODE);

        // TODO: status frames
    }

    private static void configureCollectionMotor() {
        COLLECTION_MOTOR.restoreFactoryDefaults();

        COLLECTION_MOTOR.enableVoltageCompensation(VOLTAGE_COMPENSATION_SATURATION);
        COLLECTION_MOTOR.setInverted(COLLECTION_MOTOR_INVERTED);
        COLLECTION_MOTOR.setIdleMode(DEFAULT_COLLECTION_MOTOR_IDLE_MODE);

        COLLECTION_MOTOR.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 255); // Applied output
        COLLECTION_MOTOR.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 255); // Motor movement
        COLLECTION_MOTOR.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 10000); // Motor position
        COLLECTION_MOTOR.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 10000); // Analog sensor
        COLLECTION_MOTOR.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus4, 10000); // Alternate encoder
        COLLECTION_MOTOR.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus5, 10000); // Duty cycle position
        COLLECTION_MOTOR.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus6, 10000); // Duty cycle velocity

        new Notifier(COLLECTION_MOTOR::burnFlash).startSingle(0.03);
    }
}
