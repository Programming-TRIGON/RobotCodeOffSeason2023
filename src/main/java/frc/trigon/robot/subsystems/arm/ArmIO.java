package frc.trigon.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public class ArmIO {
    @AutoLog
    protected static class ArmInputs {
        public double angleMotorPositionDegrees = 0;
        public double angleMotorVelocityDegreesPerSecond = 0;
        public double angleMotorCurrent = 0;
        public double angleMotorAppliedVoltage = 0;

        public double elevatorMotorPositionRevolutions = 0;
        public double elevatorMotorVelocityRevolutionsPerSecond = 0;
        public double elevatorMotorCurrent = 0;
        public double elevatorMotorAppliedVoltage = 0;
    }

    /**
     * Updates the inputs of the arm.
     *
     * @param inputs the inputs class to update
     */
    protected void updateInputs(ArmInputsAutoLogged inputs) {
    }

    /**
     * Sets the target angle of the angle motor.
     *
     * @param angle       the target angle
     * @param feedforward the feedforward
     */
    protected void setTargetAngle(Rotation2d angle, double feedforward) {
    }

    /**
     * Sets the target position of the elevator.
     *
     * @param position    the position in meters
     * @param feedforward the feedforward
     */
    protected void setTargetElevatorPosition(double position, double feedforward) {
    }

    /**
     * Sets whether the arm is in brake mode or not.
     *
     * @param brake whether the arm is in brake mode or not
     */
    protected void setNeutralMode(boolean brake) {
    }

    /**
     * Stops the angle motor.
     */
    protected void stopAngleMotor() {
    }

    /**
     * Stops the elevator motor.
     */
    protected void stopElevatorMotor() {
    }

    /**
     * Stops the arm.
     */
    void stop() {
        stopAngleMotor();
        stopElevatorMotor();
    }
}
