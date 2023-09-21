package frc.trigon.robot.subsystems.roller.kablamaroller;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import frc.trigon.robot.subsystems.roller.RollerIO;
import frc.trigon.robot.subsystems.roller.RollerInputsAutoLogged;
import frc.trigon.robot.utilities.Conversions;

public class KablamaRollerIO extends RollerIO {
    private final CANSparkMax collectionMotor = KablamaRollerConstants.COLLECTION_MOTOR;
    private final WPI_TalonSRX angleMotor = KablamaRollerConstants.ANGLE_MOTOR;

    @Override
    protected void updateInputs(RollerInputsAutoLogged inputs) {
//        inputs.angleMotorForwardLimitSwitchPressed = KablamaRollerConstants.FORWARD_ANGLE_LIMIT_SWITCH.get();
        inputs.angleMotorBackwardLimitSwitchPressed = !KablamaRollerConstants.REVERSE_ANGLE_LIMIT_SWITCH.get();
        inputs.angleMotorCurrent = angleMotor.getStatorCurrent();
        inputs.angleMotorAppliedVoltage = angleMotor.getBusVoltage();
        inputs.angleMotorPower = angleMotor.getMotorOutputPercent() / 100;

        inputs.collectionMotorCurrent = collectionMotor.getOutputCurrent();
        inputs.collectionMotorAppliedVoltage = collectionMotor.getBusVoltage();
        inputs.collectionMotorPower = Conversions.voltageToCompensatedPower(inputs.collectionMotorAppliedVoltage, KablamaRollerConstants.VOLTAGE_COMPENSATION_SATURATION);
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
        return angleMotor.isFwdLimitSwitchClosed() == 1;
    }

    private boolean isBackwardAngleLimitSwitchPressed() {
        return angleMotor.isRevLimitSwitchClosed() == 1;
    }
}
