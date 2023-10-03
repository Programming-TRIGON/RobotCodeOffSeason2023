package frc.trigon.robot.subsystems.roller;

import org.littletonrobotics.junction.AutoLog;

public class RollerIO {
    @AutoLog
    protected static class RollerInputs {
        public double angleMotorCurrent = 0;
//        public double angleMotorAppliedVoltage = 0;
        public double angleMotorPower = 0;
        public boolean angleMotorForwardLimitSwitchPressed = false;
        public boolean angleMotorReverseLimitSwitchPressed = false;

        public double collectionMotorCurrent = 0;
//        public double collectionMotorAppliedVoltage = 0;
        public double collectionMotorPower = 0;
    }

    /**
     * Updates the inputs of the roller.
     *
     * @param inputs the inputs class to update
     */
    protected void updateInputs(RollerInputsAutoLogged inputs) {
    }

    /**
     * Sets the target power of the roller angle motor.
     *
     * @param power the target power
     */
    protected void setTargetAnglePower(double power) {
    }

    /**
     * Sets the target power of the roller collection motor.
     *
     * @param power the target power
     */
    protected void setTargetCollectionPower(double power) {
    }

    /**
     * Stops the angle motor.
     */
    protected void stopAngleMotor() {
    }

    /**
     * Stops the collection motor.
     */
    protected void stopCollectionMotor() {
    }
}
