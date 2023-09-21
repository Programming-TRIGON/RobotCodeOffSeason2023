package frc.trigon.robot.subsystems.sideshooter;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public class SideShooterIO {
    @AutoLog
    protected static class SideShooterInputs {
        public double angleMotorPositionDegrees = 0;
        public double angleMotorVelocityDegreesPerSecond = 0;
        public double angleMotorCurrent = 0;
        public double angleMotorAppliedVoltage = 0;

        public double shootingMotorCurrent = 0;
        public double shootingMotorAppliedVoltage = 0;
        public double shootingMotorPower = 0;
    }

    /**
     * Updates the inputs of the side shooter.
     *
     * @param inputs the inputs class to update
     */
    protected void updateInputs(SideShooterInputsAutoLogged inputs) {
    }

    /**
     * Sets whether the angle motor is in brake mode or not.
     *
     * @param brake whether the angle is in brake mode or not
     */
    protected void setNeutralMode(boolean brake) {
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
     * Sets the target voltage of the shooting motor.
     *
     * @param voltage the target voltage
     */
    protected void setTargetShootingPower(double voltage) {
    }

    /**
     * Stops the angle motor.
     */
    protected void stopAngleMotor() {
    }

    /**
     * Stops the shooting motor.
     */
    protected void stopShootingMotor() {
    }
}
