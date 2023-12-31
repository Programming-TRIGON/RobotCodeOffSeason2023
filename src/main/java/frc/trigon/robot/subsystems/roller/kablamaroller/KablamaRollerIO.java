package frc.trigon.robot.subsystems.roller.kablamaroller;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import frc.trigon.robot.subsystems.roller.RollerIO;
import frc.trigon.robot.subsystems.roller.RollerInputsAutoLogged;

public class KablamaRollerIO extends RollerIO {
    private final CANSparkMax
            angleMotor = KablamaRollerConstants.ANGLE_MOTOR,
            collectionMotor = KablamaRollerConstants.COLLECTION_MOTOR;

    @Override
    protected void updateInputs(RollerInputsAutoLogged inputs) {
        inputs.angleMotorCurrent = angleMotor.getOutputCurrent();
        inputs.angleMotorAppliedVoltage = angleMotor.getBusVoltage();
        inputs.angleMotorPower = inputs.angleMotorAppliedVoltage / KablamaRollerConstants.VOLTAGE_COMPENSATION_SATURATION;
        inputs.angleMotorForwardLimitSwitchPressed = isForwardAngleLimitSwitchPressed();
        inputs.angleMotorBackwardLimitSwitchPressed = isBackwardAngleLimitSwitchPressed();

        inputs.collectionMotorCurrent = collectionMotor.getOutputCurrent();
        inputs.collectionMotorAppliedVoltage = collectionMotor.getBusVoltage();
        inputs.collectionMotorPower = inputs.collectionMotorAppliedVoltage / KablamaRollerConstants.VOLTAGE_COMPENSATION_SATURATION;
    }

    @Override
    protected void setTargetAnglePower(double power) {
        angleMotor.set(power);
    }

    @Override
    protected void setTargetCollectionPower(double power) {
        collectionMotor.set(power);
    }

    @Override
    protected void stopAngleMotor() {
        angleMotor.stopMotor();
    }

    @Override
    protected void stopCollectionMotor() {
        collectionMotor.stopMotor();
    }

    private boolean isForwardAngleLimitSwitchPressed() {
        return angleMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).isPressed();
    }

    private boolean isBackwardAngleLimitSwitchPressed() {
        return angleMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).isPressed();
    }
}
