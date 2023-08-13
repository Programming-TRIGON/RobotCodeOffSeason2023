package frc.trigon.robot.subsystems.arm.simulationarm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.arm.ArmIO;
import frc.trigon.robot.subsystems.arm.ArmInputsAutoLogged;
import frc.trigon.robot.utilities.Conversions;

public class SimulationArmIO extends ArmIO {
    private final ElevatorSim elevatorSimulation = SimulationArmConstants.ELEVATOR_SIMULATION;
    private final SingleJointedArmSim angleSimulation = SimulationArmConstants.ANGLE_SIMULATION;

    private double elevatorAppliedVoltage, angleAppliedVoltage;

    @Override
    protected void updateInputs(ArmInputsAutoLogged inputs) {
        angleSimulation.update(RobotConstants.PERIODIC_TIME_SECONDS);
        elevatorSimulation.update(RobotConstants.PERIODIC_TIME_SECONDS);

        inputs.angleMotorPositionDegrees = getAngle().getDegrees();
        inputs.angleMotorVelocityDegreesPerSecond = Units.radiansToDegrees(angleSimulation.getVelocityRadPerSec());
        inputs.angleMotorCurrent = angleSimulation.getCurrentDrawAmps();
        inputs.angleMotorAppliedVoltage = angleAppliedVoltage;

        inputs.elevatorMotorVelocityMetersPerSecond = elevatorSimulation.getVelocityMetersPerSecond();
        inputs.elevatorMotorPositionMeters = elevatorSimulation.getPositionMeters() - SimulationArmConstants.RETRACTED_ARM_LENGTH;
        inputs.elevatorMotorPositionRevolutions = Conversions.distanceToRevolutions(inputs.elevatorMotorPositionMeters, SimulationArmConstants.ELEVATOR_METERS_PER_REVOLUTION);
        inputs.elevatorMotorVelocityRevolutionsPerSecond = Conversions.distanceToRevolutions(elevatorSimulation.getVelocityMetersPerSecond(), SimulationArmConstants.ELEVATOR_METERS_PER_REVOLUTION);
        inputs.elevatorMotorCurrent = elevatorSimulation.getCurrentDrawAmps();
        inputs.elevatorMotorAppliedVoltage = elevatorAppliedVoltage;

    }

    @Override
    protected void setTargetAngle(Rotation2d angle, double feedforward) {
        final double pidOutput = SimulationArmConstants.ANGLE_PID_CONTROLLER.calculate(getAngle().getDegrees(), angle.getDegrees());
        setAngleSimulationVoltage(pidOutput + feedforward);
    }

    @Override
    protected void setTargetElevatorPosition(double position, double feedforward) {
        position = position + SimulationArmConstants.RETRACTED_ARM_LENGTH;
        final double pidOutput = SimulationArmConstants.ELEVATOR_PID_CONTROLLER.calculate(elevatorSimulation.getPositionMeters(), position);
        setElevatorSimulationVoltage(pidOutput + feedforward);
    }

    @Override
    protected void stopAngleMotor() {
        setAngleSimulationVoltage(0);
    }

    @Override
    protected void stopElevatorMotor() {
        setElevatorSimulationVoltage(0);
    }

    private Rotation2d getAngle() {
        return new Rotation2d(angleSimulation.getAngleRads());
    }

    private void setAngleSimulationVoltage(double voltage) {
        angleAppliedVoltage = MathUtil.clamp(voltage, -SimulationArmConstants.VOLTAGE_COMPENSATION_SATURATION, SimulationArmConstants.VOLTAGE_COMPENSATION_SATURATION);
        angleSimulation.setInputVoltage(angleAppliedVoltage);
    }

    private void setElevatorSimulationVoltage(double voltage) {
        elevatorAppliedVoltage = MathUtil.clamp(voltage, -SimulationArmConstants.VOLTAGE_COMPENSATION_SATURATION, SimulationArmConstants.VOLTAGE_COMPENSATION_SATURATION);
        elevatorSimulation.setInputVoltage(elevatorAppliedVoltage);
    }
}
