package frc.trigon.robot.subsystems.swerve.simulationswerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.poseestimator.PoseEstimator;
import frc.trigon.robot.subsystems.swerve.Swerve;
import frc.trigon.robot.subsystems.swerve.SwerveIO;
import frc.trigon.robot.subsystems.swerve.SwerveInputsAutoLogged;
import frc.trigon.robot.utilities.AllianceUtilities;

public class SimulationSwerveIO extends SwerveIO {
    private double simulationRadians = 0;

    @Override
    protected void updateInputs(SwerveInputsAutoLogged inputs) {
        simulationRadians += Swerve.getInstance().getCurrentVelocity().omegaRadiansPerSecond * RobotConstants.PERIODIC_TIME_SECONDS;

        inputs.gyroYawDegrees = Units.radiansToDegrees(simulationRadians);
        var x = PoseEstimator.getInstance().getCurrentPose().getX();
        var xPositionRelativeToChargeStationMiddle = x - 3.862310;
        var mid = (5.345907 - 2.598620) / 2;
        if (x > 5.345907 || x < 2.598620)
            inputs.gyroPitchDegrees = 0;
        else
            inputs.gyroPitchDegrees = PoseEstimator.getInstance().getCurrentPose().getRotation().getCos() * xPositionRelativeToChargeStationMiddle / mid * 90;
    }

    @Override
    protected void setHeading(Rotation2d heading) {
        simulationRadians = heading.getRadians();
    }
}
