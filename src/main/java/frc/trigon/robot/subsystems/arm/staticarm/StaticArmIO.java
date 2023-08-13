package frc.trigon.robot.subsystems.arm.staticarm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.subsystems.arm.ArmIO;
import frc.trigon.robot.subsystems.arm.ArmInputsAutoLogged;
import frc.trigon.robot.utilities.Conversions;

public class StaticArmIO extends ArmIO {
    private final CANSparkMax
            angleMotor = StaticArmConstants.MASTER_ANGLE_MOTOR,
            elevatorMotor = StaticArmConstants.MASTER_ELEVATOR_MOTOR;
    private final SparkMaxAbsoluteEncoder
            angleEncoder = StaticArmConstants.ANGLE_ENCODER,
            elevatorEncoder = StaticArmConstants.ELEVATOR_ENCODER;

    @Override
    protected void updateInputs(ArmInputsAutoLogged inputs) {
        inputs.angleMotorPositionDegrees = angleEncoder.getPosition();
        inputs.angleMotorVelocityDegreesPerSecond = angleEncoder.getVelocity();
        inputs.angleMotorCurrent = angleMotor.getOutputCurrent();
        inputs.angleMotorAppliedVoltage = angleMotor.getBusVoltage();

        inputs.elevatorMotorPositionRevolutions = elevatorEncoder.getPosition();
        inputs.elevatorMotorPositionMeters = Conversions.revolutionsToDistance(inputs.elevatorMotorPositionRevolutions, StaticArmConstants.ELEVATOR_METERS_PER_REVOLUTIONS);
        inputs.elevatorMotorVelocityRevolutionsPerSecond = elevatorEncoder.getVelocity();
        inputs.elevatorMotorVelocityMetersPerSecond = Conversions.revolutionsToDistance(inputs.elevatorMotorVelocityRevolutionsPerSecond, StaticArmConstants.ELEVATOR_METERS_PER_REVOLUTIONS);
        inputs.elevatorMotorCurrent = elevatorMotor.getOutputCurrent();
        inputs.elevatorMotorAppliedVoltage = elevatorMotor.getBusVoltage();
    }

    @Override
    protected void setNeutralMode(boolean brake) {
        final CANSparkMax.IdleMode idleMode = brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast;

        angleMotor.setIdleMode(idleMode);
        elevatorMotor.setIdleMode(idleMode);
    }

    @Override
    protected void setTargetAngle(Rotation2d angle, double feedforward) {
        angleMotor.getPIDController().setReference(
                angle.getDegrees(),
                CANSparkMax.ControlType.kPosition,
                0,
                feedforward,
                SparkMaxPIDController.ArbFFUnits.kVoltage
        );
    }

    @Override
    protected void setTargetElevatorPosition(double position, double feedforward) {
        final double motorRevolutions = Conversions.distanceToRevolutions(position, StaticArmConstants.ELEVATOR_METERS_PER_REVOLUTIONS);
        elevatorMotor.getPIDController().setReference(
                motorRevolutions,
                CANSparkMax.ControlType.kPosition,
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
