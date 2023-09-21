package frc.trigon.robot.subsystems.collector.simulationcollector;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.collector.CollectorIO;
import frc.trigon.robot.subsystems.collector.CollectorInputsAutoLogged;
import frc.trigon.robot.utilities.Conversions;

public class SimulationCollectorIO extends CollectorIO {
    private final FlywheelSim motorSimulation = SimulationCollectorConstants.MOTOR_SIMULATION;
    private double inputVoltage;

    @Override
    protected void updateInputs(CollectorInputsAutoLogged inputs) {
        motorSimulation.update(RobotConstants.PERIODIC_TIME_SECONDS);

        inputs.statorCurrent = motorSimulation.getCurrentDrawAmps();
        inputs.appliedVoltage = inputVoltage;
        inputs.power = Conversions.voltageToCompensatedPower(inputVoltage, SimulationCollectorConstants.VOLTAGE_COMPENSATION_SATURATION);
    }

    @Override
    protected void setTargetPower(double power) {
        final double voltage = Conversions.compensatedPowerToVoltage(power, SimulationCollectorConstants.VOLTAGE_COMPENSATION_SATURATION);
        setMotorSimulationVoltage(voltage);
    }

    @Override
    protected void stop() {
        setMotorSimulationVoltage(0);
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
