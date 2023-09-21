package frc.trigon.robot.subsystems.swerve.kablamaswerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.subsystems.swerve.SwerveModuleIO;
import frc.trigon.robot.subsystems.swerve.SwerveModuleInputsAutoLogged;
import frc.trigon.robot.utilities.Conversions;

public class KablamaSwerveModuleIO extends SwerveModuleIO {
    private final TalonFX driveMotor;
    private final CANSparkMax steerMotor;
    private final SparkMaxAbsoluteEncoder steerEncoder;

    private final StatusSignal<Double> positionSignal, velocitySignal, dutyCycleSignal;

    KablamaSwerveModuleIO(KablamaSwerveModuleConstants.KablamaSwerveModules module) {
        super(module.name());
        final KablamaSwerveModuleConstants moduleConstants = module.swerveModuleConstants;

        this.steerMotor = moduleConstants.steerMotor;
        this.driveMotor = moduleConstants.driveMotor;
        steerEncoder = steerMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

        positionSignal = driveMotor.getPosition();
        velocitySignal = driveMotor.getVelocity();
        dutyCycleSignal = driveMotor.getDutyCycle();
    }

    @Override
    protected void updateInputs(SwerveModuleInputsAutoLogged inputs) {
        inputs.steerAngleDegrees = steerEncoder.getPosition();
        inputs.steerAppliedVoltage = steerMotor.getBusVoltage();

        inputs.drivePositionRevolutions = positionSignal.refresh().getValue();
        inputs.driveDistanceMeters = Conversions.revolutionsToDistance(inputs.drivePositionRevolutions, KablamaSwerveModuleConstants.WHEEL_DIAMETER_METERS);
        inputs.driveVelocityRevolutionsPerSecond = velocitySignal.refresh().getValue();
        inputs.driveVelocityMetersPerSecond = Conversions.revolutionsToDistance(inputs.driveVelocityRevolutionsPerSecond, KablamaSwerveModuleConstants.WHEEL_DIAMETER_METERS);
        inputs.driveAppliedVoltage = Conversions.compensatedPowerToVoltage(dutyCycleSignal.refresh().getValue(), KablamaSwerveModuleConstants.VOLTAGE_COMPENSATION_SATURATION);
    }

    @Override
    protected void setTargetOpenLoopVelocity(double velocity) {
        double power = velocity / KablamaSwerveModuleConstants.MAX_THEORETICAL_SPEED_METERS_PER_SECOND;
        double voltage = Conversions.compensatedPowerToVoltage(power, KablamaSwerveModuleConstants.VOLTAGE_COMPENSATION_SATURATION);
        final VoltageOut request = new VoltageOut(
                voltage, KablamaSwerveModuleConstants.DRIVE_MOTOR_FOC, false
        );

        driveMotor.setControl(request);
    }

    @Override
    protected void setTargetClosedLoopVelocity(double velocity) {
        final double driveMotorVelocityMeters = Conversions.systemToMotor(velocity, KablamaSwerveModuleConstants.DRIVE_GEAR_RATIO);
        final double driverMotorVelocityRevolutions = Conversions.distanceToRevolutions(driveMotorVelocityMeters, KablamaSwerveModuleConstants.WHEEL_DIAMETER_METERS);
        final double feedforward = KablamaSwerveModuleConstants.DRIVE_FEEDFORWARD.calculate(velocity);
        final VelocityVoltage velocityVoltage = new VelocityVoltage(
                driverMotorVelocityRevolutions, KablamaSwerveModuleConstants.DRIVE_MOTOR_FOC,
                feedforward, 0, false
        );

        driveMotor.setControl(velocityVoltage);
    }

    @Override
    protected void setTargetAngle(Rotation2d angle) {
        steerMotor.getPIDController().setReference(angle.getDegrees(), CANSparkMax.ControlType.kPosition);
    }

    @Override
    protected void stop() {
        steerMotor.stopMotor();
        driveMotor.stopMotor();
    }

    @Override
    protected void setBrake(boolean brake) {
        final MotorOutputConfigs driveMotorOutputConfig = new MotorOutputConfigs();

        driveMotor.getConfigurator().refresh(driveMotorOutputConfig);
        driveMotorOutputConfig.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        driveMotor.getConfigurator().apply(driveMotorOutputConfig);
    }
}
