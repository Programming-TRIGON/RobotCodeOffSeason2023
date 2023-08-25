package frc.trigon.robot.subsystems.sideshooter.kablamasideshooter;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.subsystems.sideshooter.SideShooterIO;
import frc.trigon.robot.subsystems.sideshooter.SideShooterInputsAutoLogged;

public class KablamaSideShooterIO extends SideShooterIO {
    private final WPI_TalonSRX shootingMotor = KablamaSideShooterConstants.SHOOTING_MOTOR;
    private final CANSparkMax angleMotor = KablamaSideShooterConstants.ANGLE_MOTOR;
    private final SparkMaxAbsoluteEncoder angleEncoder = KablamaSideShooterConstants.ANGLE_ENCODER;

    @Override
    protected void updateInputs(SideShooterInputsAutoLogged inputs) {
        inputs.angleMotorPositionDegrees = angleEncoder.getPosition();
        inputs.angleMotorVelocityDegreesPerSecond = angleEncoder.getVelocity();
        inputs.angleMotorCurrent = angleMotor.getOutputCurrent();
        inputs.angleMotorAppliedVoltage = angleMotor.getBusVoltage();

        inputs.shootingMotorCurrent = shootingMotor.getStatorCurrent();
        inputs.shootingMotorAppliedVoltage = shootingMotor.getMotorOutputVoltage();
        inputs.shootingMotorPower = shootingMotor.getMotorOutputPercent() / 100;
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
        shootingMotor.set(power);
    }

    @Override
    protected void stopAngleMotor() {
        angleMotor.stopMotor();
    }
}
