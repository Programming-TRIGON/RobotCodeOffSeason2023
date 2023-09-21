package frc.trigon.robot.subsystems.arm.kablamaarm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.trigon.robot.subsystems.arm.ArmIO;
import frc.trigon.robot.subsystems.arm.ArmInputsAutoLogged;
import frc.trigon.robot.utilities.SRXMagEncoder;
import org.littletonrobotics.junction.Logger;

public class KablamaArmIO extends ArmIO {
    private final CANSparkMax
            angleMotor = KablamaArmConstants.MASTER_ANGLE_MOTOR,
            elevatorMotor = KablamaArmConstants.MASTER_ELEVATOR_MOTOR;
    private final SRXMagEncoder
            angleEncoder = KablamaArmConstants.ANGLE_ENCODER,
            elevatorEncoder = KablamaArmConstants.ELEVATOR_ENCODER;
    private ArmInputsAutoLogged lastInputs = new ArmInputsAutoLogged();

    @Override
    protected void updateInputs(ArmInputsAutoLogged inputs) {
        inputs.angleMotorPositionDegrees = angleEncoder.getPosition();
        inputs.angleMotorVelocityDegreesPerSecond = angleEncoder.getVelocity();
        inputs.angleMotorCurrent = angleMotor.getOutputCurrent();
        inputs.angleMotorAppliedVoltage = angleMotor.getBusVoltage();

        inputs.elevatorMotorPositionRevolutions = elevatorEncoder.getPosition();
        inputs.elevatorMotorVelocityRevolutionsPerSecond = elevatorEncoder.getVelocity();
        inputs.elevatorMotorCurrent = elevatorMotor.getOutputCurrent();
        inputs.elevatorMotorAppliedVoltage = elevatorMotor.getBusVoltage();

        lastInputs = inputs;
    }

    @Override
    protected void setNeutralMode(boolean brake) {
        final CANSparkMax.IdleMode idleMode = brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast;

        angleMotor.setIdleMode(idleMode);
        KablamaArmConstants.FOLLOWER_ANGLE_MOTOR.setIdleMode(idleMode);
        elevatorMotor.setIdleMode(idleMode);
        KablamaArmConstants.FOLLOWER_ELEVATOR_MOTOR.setIdleMode(idleMode);
    }

    @Override
    protected void setTargetAngle(Rotation2d angle, double feedforward) {
        final double pidOutput = KablamaArmConstants.ANGLE_PID_CONTROLLER.calculate(Units.degreesToRadians(lastInputs.angleMotorPositionDegrees), angle.getRadians());
        Logger.getInstance().recordOutput("pidOut", pidOutput);
        Logger.getInstance().recordOutput("ff", feedforward);
        angleMotor.getPIDController().setReference(
                pidOutput,
                CANSparkMax.ControlType.kVoltage,
                0,
                feedforward,
                SparkMaxPIDController.ArbFFUnits.kVoltage
        );
    }

    @Override
    protected void setTargetElevatorPosition(double position, double feedforward) {
        final double pidOutput = KablamaArmConstants.ELEVATOR_PID_CONTROLLER.calculate(lastInputs.elevatorMotorPositionRevolutions, position);
        elevatorMotor.getPIDController().setReference(
                pidOutput,
                CANSparkMax.ControlType.kVoltage,
                0,
                feedforward,
                SparkMaxPIDController.ArbFFUnits.kVoltage
        );
    }

    @Override
    protected void stopAngleMotor() {
        angleMotor.stopMotor();
    }

    @Override
    protected void stopElevatorMotor() {
        elevatorMotor.stopMotor();
    }
}
