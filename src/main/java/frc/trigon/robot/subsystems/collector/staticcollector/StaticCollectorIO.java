package frc.trigon.robot.subsystems.collector.staticcollector;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.trigon.robot.subsystems.collector.CollectorIO;
import frc.trigon.robot.subsystems.collector.CollectorInputsAutoLogged;

public class StaticCollectorIO extends CollectorIO {
    private final WPI_TalonSRX motor = StaticCollectorConstants.MOTOR;

    @Override
    protected void updateInputs(CollectorInputsAutoLogged inputs) {
        inputs.statorCurrent = motor.getStatorCurrent();
        inputs.appliedVoltage = motor.getMotorOutputVoltage();
        inputs.power = motor.getMotorOutputPercent() / 100;
    }

    @Override
    protected void setTargetPower(double power) {
        motor.set(power);
    }
}
