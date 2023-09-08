package frc.trigon.robot.subsystems.roller.kablamaroller;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;

public class KablamaRollerConstants {
    static final double VOLTAGE_COMPENSATION_SATURATION = 12;

    private static final boolean
            ANGLE_MOTOR_INVERTED = false,
            COLLECTION_MOTOR_INVERTED = false;
    private static final int
            ANGLE_MOTOR_ID = 1,
            COLLECTION_MOTOR_ID = 1;
    private static final CANSparkMax.IdleMode
            DEFAULT_ANGLE_MOTOR_IDLE_MODE = CANSparkMax.IdleMode.kBrake,
            DEFAULT_COLLECTION_MOTOR_IDLE_MODE = CANSparkMax.IdleMode.kCoast;
    private static final int
            ANGLE_MOTOR_CURRENT_LIMIT = 30,
            COLLECTION_MOTOR_CURRENT_LIMIT = 30;

    static final CANSparkMax
            ANGLE_MOTOR = new CANSparkMax(ANGLE_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless),
            COLLECTION_MOTOR = new CANSparkMax(COLLECTION_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

    static {
        configureAngleMotor();
        configureCollectionMotor();
    }

    private static void configureAngleMotor() {
        ANGLE_MOTOR.restoreFactoryDefaults();

        ANGLE_MOTOR.enableVoltageCompensation(VOLTAGE_COMPENSATION_SATURATION);
        ANGLE_MOTOR.setInverted(ANGLE_MOTOR_INVERTED);
        ANGLE_MOTOR.setIdleMode(DEFAULT_ANGLE_MOTOR_IDLE_MODE);
        ANGLE_MOTOR.setSmartCurrentLimit(ANGLE_MOTOR_CURRENT_LIMIT);

        ANGLE_MOTOR.burnFlash();
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
