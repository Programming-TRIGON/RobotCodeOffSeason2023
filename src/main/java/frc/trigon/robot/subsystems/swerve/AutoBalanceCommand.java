// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.trigon.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.trigon.robot.subsystems.poseestimator.PoseEstimator;
import org.littletonrobotics.junction.Logger;

public class AutoBalanceCommand extends CommandBase {
    private static final double speedInchesPerSec = 20;
    private static final double positionThresholdDegrees = 3.0;
    private static final double velocityThresholdDegreesPerSec = 5;
    private static final double stoppedFinalMinTime = 0.75;

    private final Swerve swerve = Swerve.getInstance();
    private double angleDegrees;
    private double lastStoppedFinalTimestamp;

    public AutoBalanceCommand() {
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        angleDegrees = Double.POSITIVE_INFINITY;
        lastStoppedFinalTimestamp = -1000.0;
        swerve.setClosedLoop(true);
    }

    @Override
    public void execute() {
        Rotation2d heading = swerve.getHeading();
        // Calculate charge station angle and velocity
        angleDegrees =
                heading.getSin() * swerve.getPitch().getDegrees()
                        + heading.getCos() * swerve.getRoll().getDegrees();
        double angleVelocityDegreesPerSec =
                heading.getCos() * swerve.getPitchVelocity().getDegrees()
                        + heading.getSin() * swerve.getRollVelocity().getDegrees();
        boolean shouldStopTemporary =
                (angleDegrees < 0.0 && angleVelocityDegreesPerSec > velocityThresholdDegreesPerSec)
                        || (angleDegrees > 0.0
                        && angleVelocityDegreesPerSec < -velocityThresholdDegreesPerSec);
        boolean shouldStopFinal = Math.abs(angleDegrees) < positionThresholdDegrees;
        if (shouldStopFinal) {
            lastStoppedFinalTimestamp = Timer.getFPGATimestamp();
        }
        // Send velocity to drive
        if (shouldStopTemporary) {
            swerve.stop();
        } else if (Timer.getFPGATimestamp() - lastStoppedFinalTimestamp < stoppedFinalMinTime) {
            swerve.lockSwerve();
        } else {
            swerve.fieldRelativeDrive(
                    new Translation2d(Units.inchesToMeters(speedInchesPerSec) * (angleDegrees > 0.0 ? -1.0 : 1.0), 0.0),
                    new Rotation2d(),
                    false,
                    false,
                    heading
            );
        }

        // Log data
        Logger.getInstance().recordOutput("AutoBalance/AngleDegrees", angleDegrees);
        Logger.getInstance()
                .recordOutput("AutoBalance/AngleVelocityDegreesPerSec", angleVelocityDegreesPerSec);
        Logger.getInstance().recordOutput("AutoBalance/StoppedTemporary", shouldStopTemporary);
        Logger.getInstance().recordOutput("AutoBalance/StoppedFinal", shouldStopFinal);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }
}