package frc.trigon.robot.subsystems.collector.simulationcollector;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.trigon.robot.subsystems.collector.CollectorIO;
import frc.trigon.robot.subsystems.collector.CollectorInputsAutoLogged;

public class SimulationCollectorIO extends CollectorIO {
    private final FlywheelSim motorSimulation = SimulationCollectorConstants.MOTOR_SIMULATION;
    private double inputVoltage;

    @Override
    protected void updateInputs(CollectorInputsAutoLogged inputs) {
        inputs.statorCurrent = motorSimulation.getCurrentDrawAmps();
        inputs.appliedVoltage = inputVoltage;
        inputs.power = inputVoltage / SimulationCollectorConstants.VOLTAGE_COMPENSATION_SATURATION;
    }

    @Override
    protected void setTargetPower(double power) {
        setMotorSimulationVoltage(power * SimulationCollectorConstants.VOLTAGE_COMPENSATION_SATURATION);
    }

    private void setMotorSimulationVoltage(double voltage) {
        inputVoltage = MathUtil.clamp(
                voltage,
                -SimulationCollectorConstants.VOLTAGE_COMPENSATION_SATURATION,
                SimulationCollectorConstants.VOLTAGE_COMPENSATION_SATURATION
        );

        motorSimulation.setInputVoltage(inputVoltage);
    }
}
