package frc.trigon.robot.subsystems.sideshooter.kablamasideshooter;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.trigon.robot.subsystems.sideshooter.SideShooterIO;
import frc.trigon.robot.subsystems.sideshooter.SideShooterInputsAutoLogged;
import frc.trigon.robot.utilities.Conversions;
import org.littletonrobotics.junction.Logger;


public class KablamaSideShooterIO extends SideShooterIO {
    private final TalonFX shootingMotor = KablamaSideShooterConstants.SHOOTING_MOTOR;
    private final CANSparkMax angleMotor = KablamaSideShooterConstants.ANGLE_MOTOR;
    private final SparkMaxPIDController anglePIDController = angleMotor.getPIDController();

    private SideShooterInputsAutoLogged lastInputs = new SideShooterInputsAutoLogged();

    @Override
    protected void updateInputs(SideShooterInputsAutoLogged inputs) {
        inputs.angleMotorPositionDegrees = Conversions.revolutionsToDegrees(KablamaSideShooterConstants.ANGLE_POSITION_SIGNAL.refresh().getValue());
        inputs.angleMotorVelocityDegreesPerSecond = Conversions.revolutionsToDegrees(KablamaSideShooterConstants.ANGLE_VELOCITY_SIGNAL.refresh().getValue());
        inputs.angleMotorCurrent = angleMotor.getOutputCurrent();
//        inputs.angleMotorAppliedVoltage = angleMotor.getBusVoltage();

        inputs.shootingMotorCurrent = KablamaSideShooterConstants.SHOOTING_STATOR_CURRENT_SIGNAL.refresh().getValue();
//        inputs.shootingMotorPower = KablamaSideShooterConstants.SHOOTING_MOTOR_OUTPUT_PERCENT_SIGNAL.refresh().getValue();
        inputs.shootingMotorAppliedVoltage = Conversions.compensatedPowerToVoltage(KablamaSideShooterConstants.SHOOTING_MOTOR_OUTPUT_PERCENT_SIGNAL.refresh().getValue(), KablamaSideShooterConstants.VOLTAGE_COMPENSATION_SATURATION);

        lastInputs = inputs;
    }

    @Override
    protected void setNeutralMode(boolean brake) {
        final CANSparkMax.IdleMode idleMode = brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast;
        angleMotor.setIdleMode(idleMode);
    }

    @Override
    protected void setTargetAngle(Rotation2d angle, double feedforward) {
        final double pidOutput = KablamaSideShooterConstants.ANGLE_PID_CONTROLLER.calculate(Units.degreesToRadians(lastInputs.angleMotorPositionDegrees), angle.getRadians());
        anglePIDController.setReference(
                pidOutput + feedforward,
                CANSparkMax.ControlType.kVoltage
        );
    }

    @Override
    protected void setTargetShootingPower(double voltage) {
        final VoltageOut request = new VoltageOut(
                voltage,
                KablamaSideShooterConstants.SHOOTING_MOTOR_FOC,
                false
        );
        shootingMotor.setControl(request);
    }

    @Override
    protected void stopAngleMotor() {
        angleMotor.stopMotor();
    }

    @Override
    protected void stopShootingMotor() {
        shootingMotor.stopMotor();
    }
}
