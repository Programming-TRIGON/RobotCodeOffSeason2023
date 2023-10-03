package frc.trigon.robot.subsystems.arm.kablamaarm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.trigon.robot.subsystems.arm.ArmIO;
import frc.trigon.robot.subsystems.arm.ArmInputsAutoLogged;
import frc.trigon.robot.utilities.Conversions;
import frc.trigon.robot.utilities.SRXMagEncoder;

public class KablamaArmIO extends ArmIO {
    private final CANSparkMax
            angleMotor = KablamaArmConstants.MASTER_ANGLE_MOTOR,
            elevatorMotor = KablamaArmConstants.MASTER_ELEVATOR_MOTOR;
    private final SparkMaxPIDController
            masterAnglePIDController = angleMotor.getPIDController(),
            followerAnglePIDController = KablamaArmConstants.FOLLOWER_ANGLE_MOTOR.getPIDController(),
            masterElevatorPIDController = elevatorMotor.getPIDController(),
            followerElevatorPIDController = KablamaArmConstants.FOLLOWER_ELEVATOR_MOTOR.getPIDController();
    private final SRXMagEncoder elevatorEncoder = KablamaArmConstants.ELEVATOR_ENCODER;
    private ArmInputsAutoLogged lastInputs = new ArmInputsAutoLogged();
    private boolean didSetAngle = false, didSetElevatorPosition = false;

    @Override
    protected void updateInputs(ArmInputsAutoLogged inputs) {
        inputs.angleMotorPositionDegrees = Conversions.revolutionsToDegrees(KablamaArmConstants.ANGLE_POSITION_SIGNAL.refresh().getValue());
        inputs.angleMotorVelocityDegreesPerSecond = Conversions.revolutionsToDegrees(KablamaArmConstants.ANGLE_VELOCITY_SIGNAL.refresh().getValue());
        inputs.angleMotorCurrent = angleMotor.getOutputCurrent();
//        inputs.angleMotorAppliedVoltage = angleMotor.getBusVoltage();

        inputs.elevatorMotorPositionRevolutions = elevatorEncoder.getPosition();
        inputs.elevatorMotorVelocityRevolutionsPerSecond = elevatorEncoder.getVelocity();
        inputs.elevatorMotorCurrent = elevatorMotor.getOutputCurrent();
//        inputs.elevatorMotorAppliedVoltage = elevatorMotor.getBusVoltage();

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
        didSetAngle = !didSetAngle;
        if (didSetAngle)
            return;
        final double pidOutput = KablamaArmConstants.ANGLE_PID_CONTROLLER.calculate(Units.degreesToRadians(lastInputs.angleMotorPositionDegrees), angle.getRadians());
        masterAnglePIDController.setReference(
                pidOutput + feedforward,
                CANSparkMax.ControlType.kVoltage
        );
        followerAnglePIDController.setReference(
                pidOutput + feedforward,
                CANSparkMax.ControlType.kVoltage
        );
    }

    @Override
    protected void setTargetElevatorPosition(double position, double feedforward) {
        didSetElevatorPosition = !didSetElevatorPosition;
        if (didSetElevatorPosition)
            return;
        final double pidOutput = KablamaArmConstants.ELEVATOR_PID_CONTROLLER.calculate(lastInputs.elevatorMotorPositionRevolutions, position);
        masterElevatorPIDController.setReference(
                pidOutput + feedforward,
                CANSparkMax.ControlType.kVoltage
        );
        followerElevatorPIDController.setReference(
                pidOutput + feedforward,
                CANSparkMax.ControlType.kVoltage
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
