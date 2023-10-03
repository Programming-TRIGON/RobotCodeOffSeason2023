package frc.trigon.robot.subsystems.collector.kablamacollector;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.trigon.robot.subsystems.collector.CollectorIO;
import frc.trigon.robot.subsystems.collector.CollectorInputsAutoLogged;

public class KablamaCollectorIO extends CollectorIO {
    private final WPI_TalonSRX motor = KablamaCollectorConstants.MOTOR;

    @Override
    protected void updateInputs(CollectorInputsAutoLogged inputs) {
        inputs.current = motor.getSupplyCurrent();
//        inputs.appliedVoltage = motor.getMotorOutputVoltage();
        inputs.power = motor.getMotorOutputPercent() / 100;
    }

    @Override
    protected void setTargetPower(double power) {
        motor.set(power);
    }

    @Override
    protected void stop() {
        motor.stopMotor();
    }
}
