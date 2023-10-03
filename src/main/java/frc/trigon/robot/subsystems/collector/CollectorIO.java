package frc.trigon.robot.subsystems.collector;

import org.littletonrobotics.junction.AutoLog;

public class CollectorIO {
    @AutoLog
    public static class CollectorInputs {
        public double current = 0;
//        public double appliedVoltage = 0;
        public double power = 0;
    }

    /**
     * Updates the inputs of the collector.
     *
     * @param inputs the inputs class to update
     */
    protected void updateInputs(CollectorInputsAutoLogged inputs) {
    }

    /**
     * Sets the power of the collector motor.
     *
     * @param power the power
     */
    protected void setTargetPower(double power) {
    }

    /**
     * Stops the collector motor.
     */
    protected void stop() {
    }
}
