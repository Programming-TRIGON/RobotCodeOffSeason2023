package frc.trigon.robot.subsystems.collector.staticcollector;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class StaticCollectorConstants {
    static final double VOLTAGE_COMPENSATION_SATURATION = 12;

    private static final boolean INVERTED = false;
    private static final int MOTOR_ID = 1;
    private static final NeutralMode DEFAULT_MOTOR_NEUTRAL_MODE = NeutralMode.Brake;
    private static final int CURRENT_LIMIT = 30;

    static final WPI_TalonSRX MOTOR = new WPI_TalonSRX(MOTOR_ID);

    static {
        MOTOR.configFactoryDefault();

        MOTOR.configVoltageCompSaturation(VOLTAGE_COMPENSATION_SATURATION);
        MOTOR.enableVoltageCompensation(true);

        MOTOR.setInverted(INVERTED);
        MOTOR.setNeutralMode(DEFAULT_MOTOR_NEUTRAL_MODE);

        MOTOR.configPeakCurrentLimit(CURRENT_LIMIT);
        MOTOR.enableCurrentLimit(true);
    }
}
