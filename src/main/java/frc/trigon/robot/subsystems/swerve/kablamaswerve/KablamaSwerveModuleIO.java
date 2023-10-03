package frc.trigon.robot.subsystems.swerve.kablamaswerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.subsystems.swerve.SwerveModuleIO;
import frc.trigon.robot.subsystems.swerve.SwerveModuleInputsAutoLogged;
import frc.trigon.robot.utilities.Conversions;
import org.littletonrobotics.junction.Logger;

public class KablamaSwerveModuleIO extends SwerveModuleIO {
    private final TalonFX driveMotor;
    private final CANSparkMax steerMotor;
    private final SparkMaxAbsoluteEncoder steerEncoder;
    private final SparkMaxPIDController steerPIDController;
    private final StatusSignal<Double> positionSignal, velocitySignal;

    KablamaSwerveModuleIO(KablamaSwerveModuleConstants.KablamaSwerveModules module) {
        super(module.name());
        final KablamaSwerveModuleConstants moduleConstants = module.swerveModuleConstants;

        this.steerMotor = moduleConstants.steerMotor;
        this.driveMotor = moduleConstants.driveMotor;
        steerPIDController = steerMotor.getPIDController();
        steerEncoder = steerMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

        positionSignal = driveMotor.getPosition();
        velocitySignal = driveMotor.getVelocity();
    }

    @Override
    protected void updateInputs(SwerveModuleInputsAutoLogged inputs) {
        inputs.steerAngleDegrees = steerEncoder.getPosition();
        inputs.steerCurrent = steerMotor.getOutputCurrent();
//        inputs.steerAppliedOutput = steerMotor.getAppliedOutput();

        inputs.drivePositionRevolutions = Conversions.motorToSystem(positionSignal.refresh().getValue(), KablamaSwerveModuleConstants.DRIVE_GEAR_RATIO);
        inputs.driveDistanceMeters = Conversions.revolutionsToDistance(inputs.drivePositionRevolutions, KablamaSwerveModuleConstants.WHEEL_DIAMETER_METERS);
        inputs.driveVelocityRevolutionsPerSecond = Conversions.motorToSystem(velocitySignal.refresh().getValue(), KablamaSwerveModuleConstants.DRIVE_GEAR_RATIO);
        inputs.driveVelocityMetersPerSecond = Conversions.revolutionsToDistance(inputs.driveVelocityRevolutionsPerSecond, KablamaSwerveModuleConstants.WHEEL_DIAMETER_METERS);
//        inputs.driveAppliedVoltage = Conversions.compensatedPowerToVoltage(dutyCycleSignal.refresh().getValue(), KablamaSwerveModuleConstants.VOLTAGE_COMPENSATION_SATURATION);
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

        Logger.getInstance().recordOutput(getLoggingPath() + "targetVelocity", velocity);

        driveMotor.setControl(velocityVoltage);
    }

    @Override
    protected void setTargetAngle(Rotation2d angle) {
        steerPIDController.setReference(angle.getDegrees(), CANSparkMax.ControlType.kPosition);
    }

    @Override
    protected void stop() {
        steerMotor.stopMotor();
        driveMotor.stopMotor();
    }

    @Override
    protected void setBrake(boolean brake) {
//        final MotorOutputConfigs driveMotorOutputConfig = new MotorOutputConfigs();
//
//        driveMotor.getConfigurator().refresh(driveMotorOutputConfig);
//        driveMotorOutputConfig.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
//        driveMotor.getConfigurator().apply(driveMotorOutputConfig);
        driveMotor.setControl(brake ? new StaticBrake() : new CoastOut());

        steerMotor.setIdleMode(brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    }
}
