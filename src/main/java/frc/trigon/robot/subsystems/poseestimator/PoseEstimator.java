package frc.trigon.robot.subsystems.poseestimator;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.constants.CommandsConstants;
import frc.trigon.robot.robotposesources.PoseSourceConstants;
import frc.trigon.robot.robotposesources.RelativeRobotPoseSource;
import frc.trigon.robot.robotposesources.RobotPoseSource;
import frc.trigon.robot.subsystems.leds.Leds;
import frc.trigon.robot.subsystems.swerve.Swerve;
import frc.trigon.robot.utilities.AllianceUtilities;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/**
 * A class that estimates the robot's pose using a {@link SwerveDrivePoseEstimator}, and robot pose sources.
 * This pose estimator will provide you the robot's pose relative to the current driver station.
 *
 * @author Shriqui - Captain, Omer - Programing Captain
 */
public class PoseEstimator extends SubsystemBase {
    private final static PoseEstimator INSTANCE = new PoseEstimator();

    private final Swerve swerve = Swerve.getInstance();
    private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    private final Field2d field = new Field2d();
    private final List<RobotPoseSource> robotPoseSources = new ArrayList<>();
    private DriverStation.Alliance lastAlliance = DriverStation.getAlliance();
    private final Logger logger = Logger.getInstance();

    private PoseEstimator() {
        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
                swerve.getConstants().getKinematics(),
                swerve.getHeading(),
                swerve.getModulePositions(),
                new Pose2d(5, 5, new Rotation2d()),
                PoseEstimatorConstants.STATES_AMBIGUITY,
                PoseEstimatorConstants.VISION_CALCULATIONS_AMBIGUITY
        );

        putAprilTagsOnFieldWidget();
    }

    public static PoseEstimator getInstance() {
        return INSTANCE;
    }

    @Override
    public void periodic() {
        updatePoseEstimator();
        if (didAllianceChange()) {
            updateFieldWidget();
            Leds.getInstance().getDefaultCommand().cancel();
            Leds.getInstance().setDefaultCommand(AllianceUtilities.isBlueAlliance() ? CommandsConstants.BLUE_LIGHT_SABER_COMMAND : CommandsConstants.RED_LIGHT_SABER_COMMAND);
        }

        SmartDashboard.putData("field", field);
        logger.recordOutput("robotPosition", getCurrentPose());
    }

    /**
     * @return the field widget
     */
    public Field2d getField() {
        return field;
    }

    /**
     * Resets the pose estimator to the given pose, and the gyro to the given pose's heading.
     *
     * @param currentPose the pose to reset to
     */
    public void resetPose(Pose2d currentPose) {
        final Pose2d currentBluePose = AllianceUtilities.toAlliancePose(currentPose);
        swerve.setHeading(currentBluePose.getRotation());

        resetPoseEstimator(currentBluePose);
//        new Notifier(() -> resetPoseEstimator(currentBluePose)).startSingle(PoseEstimatorConstants.GYRO_UPDATE_TIME_SECONDS);
    }

    /**
     * @return the estimated pose of the robot, relative to the current driver station
     */
    public Pose2d getCurrentPose() {
        Pose2d estimatedPose = swerveDrivePoseEstimator.getEstimatedPosition();
        if (PoseEstimatorConstants.LIMIT_POSITION)
            estimatedPose = limitPosition(estimatedPose);

        return AllianceUtilities.toAlliancePose(estimatedPose);
    }

    /**
     * Adds robot pose sources to use for the pose estimator.
     *
     * @param robotPoseSources the pose sources
     */
    public void addRobotPoseSources(RobotPoseSource... robotPoseSources) {
        this.robotPoseSources.addAll(List.of(robotPoseSources));
    }

    private Pose2d limitPosition(Pose2d pose2d) {
        final Pose2d newPose = PoseEstimatorConstants.POSE_LIMITER.limitPose(pose2d);
        if (newPose.equals(pose2d))
            return pose2d;

        resetPose(AllianceUtilities.toAlliancePose(newPose));
        return newPose;
    }

    private void updateFieldWidget() {
        putAprilTagsOnFieldWidget();
        lastAlliance = DriverStation.getAlliance();
    }

    private boolean didAllianceChange() {
        return lastAlliance != DriverStation.getAlliance();
    }

    private void resetPoseEstimator(Pose2d currentPose) {
        swerveDrivePoseEstimator.resetPosition(
//                swerve.getHeading(),
                currentPose.getRotation(),
                swerve.getModulePositions(),
                currentPose
        );

        configureRelativePoseSources(currentPose);
    }

    private void configureRelativePoseSources(Pose2d currentPose) {
        for (RobotPoseSource robotPoseSource : robotPoseSources) {
            if (!(robotPoseSource instanceof RelativeRobotPoseSource))
                continue;

            final RelativeRobotPoseSource relativeRobotPoseSource = (RelativeRobotPoseSource) robotPoseSource;
            relativeRobotPoseSource.setRelativePose(currentPose);
        }
    }

    private void updatePoseEstimator() {
        updatePoseEstimatorStates();
        attemptToUpdateWithRobotPoseSources();
        field.setRobotPose(getCurrentPose());
    }

    private void attemptToUpdateWithRobotPoseSources() {
        for (RobotPoseSource robotPoseSource : robotPoseSources) {
            if (robotPoseSource.hasNewResult())
                updateFromPoseSource(robotPoseSource);
        }
    }

    private void updateFromPoseSource(RobotPoseSource robotPoseSource) {
        final Pose2d robotPose = robotPoseSource.getRobotPose();

        field.getObject(robotPoseSource.getName()).setPose(AllianceUtilities.toAlliancePose(robotPose));
        swerveDrivePoseEstimator.addVisionMeasurement(
                robotPose,
                robotPoseSource.getLastResultTimestamp()
        );
    }

    private void updatePoseEstimatorStates() {
        swerveDrivePoseEstimator.update(swerve.getHeading(), swerve.getModulePositions());
    }

    private void putAprilTagsOnFieldWidget() {
        final HashMap<Integer, Pose3d> tagsIdToPose = PoseSourceConstants.TAGS_ID_TO_POSE;

        for (Integer currentID : tagsIdToPose.keySet()) {
            final Pose2d tagPose = tagsIdToPose.get(currentID).toPose2d();
            field.getObject("Tag " + currentID).setPose(AllianceUtilities.toAlliancePose(tagPose));
        }
    }
}
