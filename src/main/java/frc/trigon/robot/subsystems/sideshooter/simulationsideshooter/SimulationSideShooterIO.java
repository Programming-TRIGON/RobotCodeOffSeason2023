package frc.trigon.robot.subsystems.sideshooter.simulationsideshooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.sideshooter.SideShooterIO;
import frc.trigon.robot.subsystems.sideshooter.SideShooterInputsAutoLogged;

public class SimulationSideShooterIO extends SideShooterIO {
    private final SingleJointedArmSim angleMotorSimulation = SimulationSideShooterConstants.ANGLE_MOTOR_SIMULATION;
    private final FlywheelSim shootingMotorSimulation = SimulationSideShooterConstants.SHOOTING_MOTOR_SIMULATION;
    private double shootingMotorInputVoltage, angleMotorInputVoltage;

    @Override
    protected void updateInputs(SideShooterInputsAutoLogged inputs) {
        angleMotorSimulation.update(RobotConstants.PERIODIC_TIME_SECONDS);
        shootingMotorSimulation.update(RobotConstants.PERIODIC_TIME_SECONDS);

        inputs.angleMotorPositionDegrees = Units.radiansToDegrees(angleMotorSimulation.getAngleRads());
        inputs.angleMotorVelocityDegreesPerSecond = Units.radiansToDegrees(angleMotorSimulation.getVelocityRadPerSec());
        inputs.angleMotorCurrent = angleMotorSimulation.getCurrentDrawAmps();
        inputs.angleMotorAppliedVoltage = angleMotorInputVoltage;

        inputs.shootingMotorCurrent = shootingMotorSimulation.getCurrentDrawAmps();
        inputs.shootingMotorAppliedVoltage = shootingMotorInputVoltage;
        inputs.shootingMotorPower = inputs.shootingMotorAppliedVoltage / SimulationSideShooterConstants.VOLTAGE_COMPENSATION_SATURATION;
    }

    @Override
    protected void setTargetAngle(Rotation2d angle, double feedforward) {
        final double pidOutput = SimulationSideShooterConstants.ANGLE_MOTOR_PID_CONTROLLER.calculate(
                angleMotorSimulation.getAngleRads(), angle.getRadians()
        );

        setAngleMotorSimulationVoltage(pidOutput + feedforward);
    }

    @Override
    protected void setTargetShootingPower(double power) {
        setShootingMotorSimulationVoltage(power * SimulationSideShooterConstants.VOLTAGE_COMPENSATION_SATURATION);
    }

    @Override
    protected void stopAngleMotor() {
        setAngleMotorSimulationVoltage(0);
    }

    private void setAngleMotorSimulationVoltage(double voltage) {
        angleMotorInputVoltage = MathUtil.clamp(
                voltage,
                -SimulationSideShooterConstants.VOLTAGE_COMPENSATION_SATURATION,
                SimulationSideShooterConstants.VOLTAGE_COMPENSATION_SATURATION
        );

        angleMotorSimulation.setInputVoltage(angleMotorInputVoltage);
    }

    private void setShootingMotorSimulationVoltage(double voltage) {
        shootingMotorInputVoltage = MathUtil.clamp(
                voltage,
                -SimulationSideShooterConstants.VOLTAGE_COMPENSATION_SATURATION,
                SimulationSideShooterConstants.VOLTAGE_COMPENSATION_SATURATION
        );

        shootingMotorSimulation.setInputVoltage(shootingMotorInputVoltage);
    }
}
