package frc.trigon.robot.subsystems.swerve.kablamaswerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.subsystems.swerve.SwerveIO;
import frc.trigon.robot.subsystems.swerve.SwerveInputsAutoLogged;

public class KablamaSwerveIO extends SwerveIO {
    private final Pigeon2 gyro = KablamaSwerveConstants.GYRO;

    @Override
    protected void updateInputs(SwerveInputsAutoLogged inputs) {
        inputs.gyroYawDegrees = KablamaSwerveConstants.YAW_SIGNAL.refresh().getValue();
        inputs.gyroPitchDegrees = -KablamaSwerveConstants.PITCH_SIGNAL.refresh().getValue();
        inputs.gyroRollDegrees = KablamaSwerveConstants.ROLL_SIGNAL.refresh().getValue();
        inputs.gyroPitchVelocity = -KablamaSwerveConstants.PITCH_VELOCITY_SIGNAL.refresh().getValue();
        inputs.gyroRollVelocity = KablamaSwerveConstants.ROLL_VELOCITY_SIGNAL.refresh().getValue();
        inputs.accelerationX = KablamaSwerveConstants.X_ACCELERATION_SIGNAL.refresh().getValue();
        inputs.accelerationY = KablamaSwerveConstants.Y_ACCELERATION_SIGNAL.refresh().getValue();
        inputs.accelerationZ = KablamaSwerveConstants.Z_ACCELERATION_SIGNAL.refresh().getValue();
    }

    @Override
    protected void setHeading(Rotation2d heading) {
        gyro.setYaw(heading.getDegrees());
    }
}
