package frc.trigon.robot.subsystems.sideshooter.kablamasideshooter;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.subsystems.sideshooter.SideShooterIO;
import frc.trigon.robot.subsystems.sideshooter.SideShooterInputsAutoLogged;

public class KablamaSideShooterIO extends SideShooterIO {
    private final TalonFX shootingMotor = KablamaSideShooterConstants.SHOOTING_MOTOR;
    private final CANSparkMax angleMotor = KablamaSideShooterConstants.ANGLE_MOTOR;
    private final SparkMaxAbsoluteEncoder angleEncoder = KablamaSideShooterConstants.ANGLE_ENCODER;

    @Override
    protected void updateInputs(SideShooterInputsAutoLogged inputs) {
        inputs.angleMotorPositionDegrees = angleEncoder.getPosition();
        inputs.angleMotorVelocityDegreesPerSecond = angleEncoder.getVelocity();
        inputs.angleMotorCurrent = angleMotor.getOutputCurrent();
        inputs.angleMotorAppliedVoltage = angleMotor.getBusVoltage();

        inputs.shootingMotorCurrent = KablamaSideShooterConstants.STATOR_CURRENT_SIGNAL.refresh().getValue();
        inputs.shootingMotorPower = KablamaSideShooterConstants.MOTOR_OUTPUT_PERCENT_SIGNAL.refresh().getValue();
        inputs.shootingMotorAppliedVoltage = inputs.shootingMotorPower * KablamaSideShooterConstants.VOLTAGE_COMPENSATION_SATURATION;
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
    protected void setTargetShootingPower(double power) {
        final VoltageOut request = new VoltageOut(
                power * KablamaSideShooterConstants.VOLTAGE_COMPENSATION_SATURATION,
                KablamaSideShooterConstants.SHOOTING_MOTOR_FOC,
                false
        );
        shootingMotor.setControl(request);
    }

    @Override
    protected void stopAngleMotor() {
        angleMotor.stopMotor();
    }
}
