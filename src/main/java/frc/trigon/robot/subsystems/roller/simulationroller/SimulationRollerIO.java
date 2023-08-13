package frc.trigon.robot.subsystems.roller.simulationroller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.roller.RollerIO;
import frc.trigon.robot.subsystems.roller.RollerInputsAutoLogged;

public class SimulationRollerIO extends RollerIO {
    private final SingleJointedArmSim angleMotorSimulation = SimulationRollerConstants.ANGLE_MOTOR_SIMULATION;
    private final FlywheelSim collectionMotorSimulation = SimulationRollerConstants.COLLECTION_MOTOR_SIMULATION;
    private double collectionMotorInputVoltage, angleMotorInputVoltage;

    @Override
    protected void updateInputs(RollerInputsAutoLogged inputs) {
        angleMotorSimulation.update(RobotConstants.PERIODIC_TIME_SECONDS);
        collectionMotorSimulation.update(RobotConstants.PERIODIC_TIME_SECONDS);

        inputs.angleMotorCurrent = angleMotorSimulation.getCurrentDrawAmps();
        inputs.angleMotorAppliedVoltage = angleMotorInputVoltage;
        inputs.angleMotorPower = angleMotorInputVoltage / SimulationRollerConstants.VOLTAGE_COMPENSATION_SATURATION;
        inputs.angleMotorForwardLimitSwitchPressed = angleMotorSimulation.hasHitUpperLimit();
        inputs.angleMotorBackwardLimitSwitchPressed = angleMotorSimulation.hasHitLowerLimit();

        inputs.collectionMotorCurrent = collectionMotorSimulation.getCurrentDrawAmps();
        inputs.collectionMotorAppliedVoltage = collectionMotorInputVoltage;
        inputs.collectionMotorPower = collectionMotorInputVoltage / SimulationRollerConstants.VOLTAGE_COMPENSATION_SATURATION;
    }

    @Override
    protected void setTargetAnglePower(double power) {
        setAngleMotorSimulationVoltage(power * SimulationRollerConstants.VOLTAGE_COMPENSATION_SATURATION);
    }

    @Override
    protected void setTargetCollectionPower(double power) {
        setCollectionMotorSimulationVoltage(power * SimulationRollerConstants.VOLTAGE_COMPENSATION_SATURATION);
    }

    @Override
    protected void stopAngleMotor() {
        setAngleMotorSimulationVoltage(0);
    }

    @Override
    protected void stopCollectionMotor() {
        setCollectionMotorSimulationVoltage(0);
    }

    private void setAngleMotorSimulationVoltage(double voltage) {
        angleMotorInputVoltage = MathUtil.clamp(
                voltage,
                -SimulationRollerConstants.VOLTAGE_COMPENSATION_SATURATION,
                SimulationRollerConstants.VOLTAGE_COMPENSATION_SATURATION
        );

        angleMotorSimulation.setInputVoltage(angleMotorInputVoltage);
    }

    private void setCollectionMotorSimulationVoltage(double voltage) {
        collectionMotorInputVoltage = MathUtil.clamp(
                voltage,
                -SimulationRollerConstants.VOLTAGE_COMPENSATION_SATURATION,
                SimulationRollerConstants.VOLTAGE_COMPENSATION_SATURATION
        );

        collectionMotorSimulation.setInputVoltage(collectionMotorInputVoltage);
    }
}
