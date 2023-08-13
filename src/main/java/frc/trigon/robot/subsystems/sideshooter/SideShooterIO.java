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
     * Sets the target angle of the angle motor.
     *
     * @param angle       the target angle
     * @param feedforward the feedforward
     */
    protected void setTargetAngle(Rotation2d angle, double feedforward) {
    }

    /**
     * Sets the target power of the shooting motor.
     *
     * @param power the target power
     */
    protected void setTargetShootingPower(double power) {
    }

    /**
     * Stops the angle motor.
     */
    protected void stopAngleMotor() {
    }
}
