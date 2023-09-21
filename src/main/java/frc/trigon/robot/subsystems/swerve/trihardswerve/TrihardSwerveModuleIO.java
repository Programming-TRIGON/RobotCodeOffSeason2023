package frc.trigon.robot.subsystems.swerve.trihardswerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.subsystems.swerve.SwerveModuleIO;
import frc.trigon.robot.subsystems.swerve.SwerveModuleInputsAutoLogged;
import frc.trigon.robot.utilities.Conversions;

public class TrihardSwerveModuleIO extends SwerveModuleIO {
    private final TalonFX steerMotor, driveMotor;
    private final StatusSignal<Double> steerPositionSignal, steerDutyCycleSignal, drivePositionSignal, driveVelocitySignal, driveDutyCycleSignal;

    TrihardSwerveModuleIO(TrihardSwerveModuleConstants.TrihardSwerveModules module) {
        super(module.name());
        final TrihardSwerveModuleConstants moduleConstants = module.swerveModuleConstants;

        this.steerMotor = moduleConstants.steerMotor;
        this.driveMotor = moduleConstants.driveMotor;

        steerPositionSignal = steerMotor.getPosition();
        steerDutyCycleSignal = steerMotor.getDutyCycle();

        drivePositionSignal = driveMotor.getPosition();
        driveVelocitySignal = driveMotor.getVelocity();
        driveDutyCycleSignal = driveMotor.getDutyCycle();
    }

    @Override
    protected void updateInputs(SwerveModuleInputsAutoLogged inputs) {
        inputs.steerAngleDegrees = getAngleDegrees();
        inputs.steerAppliedVoltage = Conversions.compensatedPowerToVoltage(steerDutyCycleSignal.refresh().getValue(), TrihardSwerveModuleConstants.VOLTAGE_COMPENSATION_SATURATION);

        inputs.drivePositionRevolutions = drivePositionSignal.refresh().getValue();
        inputs.driveDistanceMeters = driveMotorValueToDistance(inputs.drivePositionRevolutions);
        inputs.driveVelocityRevolutionsPerSecond = driveVelocitySignal.refresh().getValue();
        inputs.driveVelocityMetersPerSecond = driveMotorValueToDistance(inputs.driveVelocityRevolutionsPerSecond);
        inputs.driveAppliedVoltage = Conversions.compensatedPowerToVoltage(driveDutyCycleSignal.refresh().getValue(), TrihardSwerveModuleConstants.VOLTAGE_COMPENSATION_SATURATION);
    }

    @Override
    protected void setTargetOpenLoopVelocity(double velocity) {
        final double power = velocity / TrihardSwerveModuleConstants.MAX_THEORETICAL_SPEED_METERS_PER_SECOND;
        final double voltage = Conversions.compensatedPowerToVoltage(power, TrihardSwerveModuleConstants.VOLTAGE_COMPENSATION_SATURATION);
        final VoltageOut request = new VoltageOut(
                voltage,
                TrihardSwerveModuleConstants.DRIVE_MOTOR_FOC,
                false
        );
        driveMotor.setControl(request);
    }

    @Override
    protected void setTargetClosedLoopVelocity(double velocity) {
        final double driveMotorVelocityMeters = Conversions.systemToMotor(velocity, TrihardSwerveModuleConstants.DRIVE_GEAR_RATIO);
        final double driverMotorVelocityRevolutions = Conversions.distanceToRevolutions(driveMotorVelocityMeters, TrihardSwerveModuleConstants.WHEEL_DIAMETER_METERS);
        final double feedforward = TrihardSwerveModuleConstants.DRIVE_FEEDFORWARD.calculate(velocity);
        final VelocityVoltage velocityVoltage = new VelocityVoltage(
                driverMotorVelocityRevolutions, TrihardSwerveModuleConstants.DRIVE_MOTOR_FOC,
                feedforward, 0, false
        );

        driveMotor.setControl(velocityVoltage);
    }

    @Override
    protected void setTargetAngle(Rotation2d angle) {
        final double scopedAngle = scope(angle);
        final double motorAngle = Conversions.systemToMotor(scopedAngle, TrihardSwerveModuleConstants.STEER_GEAR_RATIO);
        final double motorRevolutions = Conversions.degreesToRevolutions(motorAngle);
        final PositionVoltage positionVoltage = new PositionVoltage(motorRevolutions);
        steerMotor.setControl(positionVoltage);
    }

    @Override
    protected void stop() {
        steerMotor.stopMotor();
        driveMotor.stopMotor();
    }

    @Override
    protected void setBrake(boolean brake) {
        final MotorOutputConfigs driveMotorOutputConfigs = new MotorOutputConfigs();

        driveMotor.getConfigurator().refresh(driveMotorOutputConfigs);
        driveMotorOutputConfigs.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        driveMotor.getConfigurator().apply(driveMotorOutputConfigs);
    }

    private double getAngleDegrees() {
        final double motorRevolutions = steerPositionSignal.refresh().getValue();
        final double motorDegrees = Conversions.revolutionsToDegrees(motorRevolutions);

        return Conversions.motorToSystem(motorDegrees, TrihardSwerveModuleConstants.STEER_GEAR_RATIO);
    }

    private double driveMotorValueToDistance(double value) {
        final double systemRevolutions = Conversions.motorToSystem(value, TrihardSwerveModuleConstants.DRIVE_GEAR_RATIO);
        return Conversions.revolutionsToDistance(systemRevolutions, TrihardSwerveModuleConstants.WHEEL_DIAMETER_METERS);
    }

    private double scope(Rotation2d targetRotation2d) {
        double currentDegrees = getAngleDegrees() % 360;
        double targetDegrees = targetRotation2d.getDegrees() % 360;
        double difference = targetDegrees - currentDegrees;
        if (difference < -180)
            difference += 360;
        else if (difference > 180)
            difference -= 360;

        return difference + getAngleDegrees();
    }
}
