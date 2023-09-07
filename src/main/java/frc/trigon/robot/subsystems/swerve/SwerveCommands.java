package frc.trigon.robot.subsystems.swerve;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.subsystems.poseestimator.PoseEstimator;
import frc.trigon.robot.utilities.AllianceUtilities;
import frc.trigon.robot.utilities.Maths;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SwerveCommands {
    private static final Swerve SWERVE = Swerve.getInstance();
    private static final PoseEstimator POSE_ESTIMATOR = RobotContainer.POSE_ESTIMATOR;

    /**
     * Constructs a command that autonomously drives the robot to a target pose, in the community zone.
     * This command will avoid hitting the charge station.
     *
     * @param constraints   the path constraints to use
     * @param targetPose    the target pose
     * @param xAdder        how much to add to the target x position on the first enter to the community zone (this should be used when you don't want the robot to touch the grid initially)
     * @param maxRepetition how many times should the path repeat to try and reach the setpoint
     * @return the command
     */
    public static CommandBase getEnterCommunityCommand(PathConstraints constraints, Pose2d targetPose, double xAdder, int maxRepetition) {
        return new ProxyCommand(() -> {
            final Pose2d currentPose = POSE_ESTIMATOR.getCurrentPose();
            final CommandBase enterCommunityCommand;

            if (isHittingChargeStationFromSideOnCommunityEnter(toTargetAlliancePose(currentPose).getTranslation(), targetPose.getTranslation()))
                enterCommunityCommand = getEnterCommunityFromSideCommand(constraints, targetPose, xAdder);
            else if (isHittingChargeStationFromFrontOnCommunityEnter(toTargetAlliancePose(currentPose).getTranslation(), targetPose.getTranslation()))
                enterCommunityCommand = getEnterCommunityFromFrontCommand(constraints, targetPose, xAdder);
            else return getDriveToPoseCommand(constraints, () -> targetPose, true, maxRepetition);

            return enterCommunityCommand.andThen(getDriveToPoseCommand(constraints, () -> targetPose, true, maxRepetition - 1));
        });
    }

    /**
     * Creates a command that will drive the robot using the given path group and event map.
     *
     * @param pathGroup the path group to follow
     * @param eventMap  the event map to use
     * @return the command
     */
    public static SequentialCommandGroup getFollowPathGroupCommand(List<PathPlannerTrajectory> pathGroup, Map<String, Command> eventMap, boolean useAllianceColor) {
        final Command initializeDriveAndShowTargetCommand = new InstantCommand(() -> {
            initializeDrive(true);
        });
        final SwerveAutoBuilder swerveAutoBuilder = new SwerveAutoBuilder(
                POSE_ESTIMATOR::getCurrentPose,
                (pose) -> {
                },
                SWERVE.getConstants().getKinematics(),
                SWERVE.getConstants().getTranslationPIDConstants(),
                SWERVE.getConstants().getAutoRotationPIDConstants(),
                SWERVE::setTargetModuleStates,
                eventMap,
                useAllianceColor,
                SWERVE
        );

        return initializeDriveAndShowTargetCommand.andThen(swerveAutoBuilder.fullAuto(pathGroup));
    }

    /**
     * Creates a command that will drive the robot to the target pose, as proxy.
     *
     * @param constraints        the path constraints to use
     * @param targetPoseSupplier the target pose supplier
     * @param useAllianceColor   whether to use the alliance color
     * @param maxRepetition      how many times should the path repeat to try and reach the setpoint
     * @return the command
     */
    public static ProxyCommand getDriveToPoseCommand(PathConstraints constraints, Supplier<Pose2d> targetPoseSupplier, boolean useAllianceColor, int maxRepetition) {
        return new ProxyCommand(() -> getDriveToPoseCommand(constraints, targetPoseSupplier.get(), useAllianceColor, maxRepetition));
    }

    /**
     * Creates a command that will drive the robot to the target pose.
     *
     * @param constraints      the path constraints to use
     * @param targetPose       the target pose
     * @param useAllianceColor whether to use the alliance color
     * @param maxRepetition    how many times should the path repeat to try and reach the setpoint
     * @return the command
     */
    public static CommandBase getDriveToPoseCommand(PathConstraints constraints, Pose2d targetPose, boolean useAllianceColor, int maxRepetition) {
        final Pose2d targetAlliancePose = useAllianceColor ? toTargetAlliancePose(targetPose) : targetPose;
        final Pose2d currentPose = POSE_ESTIMATOR.getCurrentPose();
        final Rotation2d angleBetweenTranslations = Maths.getAngleBetweenTranslations(
                currentPose.getTranslation(),
                targetAlliancePose.getTranslation()
        );
        final PathPoint currentPoint = new PathPoint(
                currentPose.getTranslation(),
                angleBetweenTranslations,
                currentPose.getRotation(),
                SWERVE.getCurrentVelocity().vxMetersPerSecond
        );
        final PathPoint targetPoint = new PathPoint(
                targetAlliancePose.getTranslation(),
                angleBetweenTranslations,
                targetAlliancePose.getRotation()
        );
        final PathPlannerTrajectory path = PathPlanner.generatePath(constraints, currentPoint, targetPoint);

        if (maxRepetition == 1 || SWERVE.atPose(targetAlliancePose))
            return getFollowPathGroupCommand(List.of(path), new HashMap<>(), false);

        return new SequentialCommandGroup(
                getFollowPathGroupCommand(List.of(path), new HashMap<>(), false),
                new ProxyCommand(() -> getDriveToPoseCommand(constraints, targetPose, useAllianceColor, maxRepetition - 1))
        );
    }

    /**
     * Constructs a command that drives the x-axis of the swerve with the given power, relative to the field's frame of reference.
     * This command will lock the y-axis and angle.
     *
     * @param xSupplier  the target x-axis power supplier
     * @param yAxisLock  the y position to lock the robot to
     * @param angleLock  the angle to lock the robot to
     * @param rateLimitX whether to rate limit the x-axis (no need for y-axis since it's locked)
     * @return the command
     */
    public static FunctionalCommand getClosedLoopXAxisDriveCommand(DoubleSupplier xSupplier, double yAxisLock, Rotation2d angleLock, boolean rateLimitX) {
        return new FunctionalCommand(
                () -> {
                    initializeDrive(true);
                    SWERVE.resetYAxisController();
                    SWERVE.resetRotationController();
                },
                () -> SWERVE.xAxisDrive(xSupplier.getAsDouble(), yAxisLock, angleLock, rateLimitX),
                (interrupted) -> stopDrive(),
                () -> false,
                SWERVE
        );
    }

    /**
     * Creates a command that drives the swerve with the given powers, relative to the field's frame of reference, in closed loop mode.
     *
     * @param xSupplier     the target forwards power
     * @param ySupplier     the target leftwards power
     * @param thetaSupplier the target theta power, CCW+
     * @return the command
     */
    public static FunctionalCommand getClosedLoopFieldRelativeDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier, boolean rateLimit) {
        return new FunctionalCommand(
                () -> initializeDrive(true),
                () -> SWERVE.fieldRelativeDrive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble(), rateLimit),
                (interrupted) -> stopDrive(),
                () -> false,
                SWERVE
        );
    }

    /**
     * Creates a command that drives the swerve with the given powers, relative to the field's frame of reference, in closed loop mode.
     * This command will use pid to reach the target angle.
     *
     * @param xSupplier     the target forwards power
     * @param ySupplier     the target leftwards power
     * @param angleSupplier the target angle supplier
     * @return the command
     */
    public static FunctionalCommand getClosedLoopFieldRelativeDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, Supplier<Rotation2d> angleSupplier, boolean rateLimit) {
        return new FunctionalCommand(
                () -> {
                    initializeDrive(true);
                    SWERVE.resetRotationController();
                },
                () -> SWERVE.fieldRelativeDrive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), angleSupplier.get(), rateLimit),
                (interrupted) -> stopDrive(),
                () -> false,
                SWERVE
        );
    }

    /**
     * Creates a command that drives the swerve with the given powers, relative to the field's frame of reference, in closed open mode.
     *
     * @param xSupplier     the target forwards power
     * @param ySupplier     the target leftwards power
     * @param thetaSupplier the target theta power, CCW+
     * @return the command
     */
    public static FunctionalCommand getOpenLoopFieldRelativeDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier, boolean rateLimit) {
        return new FunctionalCommand(
                () -> initializeDrive(false),
                () -> SWERVE.fieldRelativeDrive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble(), rateLimit),
                (interrupted) -> stopDrive(),
                () -> false,
                SWERVE
        );
    }

    /**
     * Creates a command that drives the swerve with the given powers, relative to the field's frame of reference, in open loop mode.
     * This command will use pid to reach the target angle.
     *
     * @param xSupplier     the target forwards power
     * @param ySupplier     the target leftwards power
     * @param angleSupplier the target angle supplier
     * @return the command
     */
    public static FunctionalCommand getOpenLoopFieldRelativeDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, Supplier<Rotation2d> angleSupplier, boolean rateLimit) {
        return new FunctionalCommand(
                () -> {
                    initializeDrive(false);
                    SWERVE.resetRotationController();
                },
                () -> SWERVE.fieldRelativeDrive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), angleSupplier.get(), rateLimit),
                (interrupted) -> stopDrive(),
                () -> false,
                SWERVE
        );
    }

    /**
     * Creates a command that drives the swerve with the given powers, relative to the robot's shooter frame of reference, in closed loop mode.
     * This is like self relative drive, but with a change to the robot's forward angle.
     *
     * @param xSupplier     the target forwards power
     * @param ySupplier     the target leftwards power
     * @param thetaSupplier the target theta power, CCW+
     * @return the command
     */
    public static FunctionalCommand getClosedLoopShooterRelativeDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier, boolean rateLimit) {
        return new FunctionalCommand(
                () -> initializeDrive(true),
                () -> SWERVE.fieldRelativeDrive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble(), rateLimit, Rotation2d.fromDegrees(-90)),
                (interrupted) -> stopDrive(),
                () -> false,
                SWERVE
        );
    }

    /**
     * Creates a command that drives the swerve with the given powers, relative to the robot's shooter frame of reference, in open loop mode.
     * This is like self relative drive, but with a change to the robot's forward angle.
     *
     * @param xSupplier     the target forwards power
     * @param ySupplier     the target leftwards power
     * @param thetaSupplier the target theta power, CCW+
     * @return the command
     */
    public static FunctionalCommand getOpenLoopShooterRelativeDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier, boolean rateLimit) {
        return new FunctionalCommand(
                () -> initializeDrive(false),
                () -> SWERVE.fieldRelativeDrive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble(), rateLimit, Rotation2d.fromDegrees(-90)),
                (interrupted) -> stopDrive(),
                () -> false,
                SWERVE
        );
    }

    /**
     * Creates a command that drives the swerve with the given powers, relative to the robot's frame of reference, in closed loop mode.
     *
     * @param xSupplier     the target forwards power
     * @param ySupplier     the target leftwards power
     * @param thetaSupplier the target theta power, CCW+
     * @return the command
     */
    public static FunctionalCommand getClosedLoopSelfRelativeDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier, boolean rateLimit) {
        return new FunctionalCommand(
                () -> initializeDrive(true),
                () -> SWERVE.selfRelativeDrive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble(), rateLimit),
                (interrupted) -> stopDrive(),
                () -> false,
                SWERVE
        );
    }

    /**
     * Creates a command that drives the swerve with the given powers, relative to the robot's frame of reference, in open loop mode.
     *
     * @param xSupplier     the target forwards power
     * @param ySupplier     the target leftwards power
     * @param thetaSupplier the target theta power, CCW+
     * @return the command
     */
    public static FunctionalCommand getOpenLoopSelfRelativeDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier, boolean rateLimit) {
        return new FunctionalCommand(
                () -> initializeDrive(false),
                () -> SWERVE.selfRelativeDrive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble(), rateLimit),
                (interrupted) -> stopDrive(),
                () -> false,
                SWERVE
        );
    }

    private static CommandBase getEnterCommunityFromSideCommand(PathConstraints constraints, Pose2d targetPose, double xAdder) {
        final Supplier<Pose2d> driveToXPose = () -> new Pose2d(
                targetPose.getX() + xAdder,
                POSE_ESTIMATOR.getCurrentPose().getY(),
                targetPose.getRotation()
        );
        final CommandBase driveToXCommand = getDriveToPoseCommand(constraints, driveToXPose, false, 1);
        final CommandBase driveToPoseCommand = getDriveToPoseCommand(constraints, () -> targetPose, true, 1);
        return driveToXCommand.andThen(driveToPoseCommand);
    }

    private static CommandBase getEnterCommunityFromFrontCommand(PathConstraints constraints, Pose2d targetPose, double xAdder) {
        final Pose2d currentPose = POSE_ESTIMATOR.getCurrentPose();
        final double closestY = Maths.getClosestNumber(targetPose.getY(), FieldConstants.CHARGE_STATION_HIGHER_Y_FROM_ROBOT, FieldConstants.CHARGE_STATION_LOWER_Y_FROM_ROBOT);
        final Pose2d driveToYPose = new Pose2d(
                currentPose.getX(),
                closestY,
                targetPose.getRotation()
        );
        final CommandBase driveToYCommand = getDriveToPoseCommand(constraints, () -> driveToYPose, true, 1);
        return driveToYCommand.andThen(getEnterCommunityFromSideCommand(constraints, targetPose, xAdder));
    }

    private static boolean isHittingChargeStationFromSideOnCommunityEnter(Translation2d currentPosition, Translation2d targetPosition) {
        if (currentPosition.getX() < FieldConstants.CHARGE_STATION_INNER_X_FROM_ROBOT || targetPosition.getX() > FieldConstants.CHARGE_STATION_OUTER_X_FROM_ROBOT)
            return false;
        if (currentPosition.getY() < FieldConstants.CHARGE_STATION_HIGHER_Y_FROM_ROBOT && currentPosition.getY() > FieldConstants.CHARGE_STATION_LOWER_Y_FROM_ROBOT)
            return false;
        return !(targetPosition.getY() > FieldConstants.CHARGE_STATION_HIGHER_Y_FROM_ROBOT) && !(targetPosition.getY() < FieldConstants.CHARGE_STATION_LOWER_Y_FROM_ROBOT);
    }

    private static boolean isHittingChargeStationFromFrontOnCommunityEnter(Translation2d currentPosition, Translation2d targetPosition) {
        if (currentPosition.getX() < FieldConstants.CHARGE_STATION_INNER_X_FROM_ROBOT || targetPosition.getX() > FieldConstants.CHARGE_STATION_OUTER_X_FROM_ROBOT)
            return false;
        return !(currentPosition.getY() > FieldConstants.CHARGE_STATION_HIGHER_Y_FROM_ROBOT) && !(currentPosition.getY() < FieldConstants.CHARGE_STATION_LOWER_Y_FROM_ROBOT);
    }

    private static Pose2d toTargetAlliancePose(Pose2d pose2d) {
        if (AllianceUtilities.isBlueAlliance())
            return pose2d;

        return new Pose2d(
                pose2d.getX(),
                FieldConstants.FIELD_WIDTH_METERS - pose2d.getY(),
                pose2d.getRotation()
        );
    }

    private static Pose2d getHolonomicPose(PathPlannerTrajectory.PathPlannerState state) {
        return new Pose2d(state.poseMeters.getTranslation(), state.holonomicRotation);
    }

    private static void initializeDrive(boolean closedLoop) {
        SWERVE.setBrake(true);
        SWERVE.setClosedLoop(closedLoop);
    }

    private static void stopDrive() {
        SWERVE.stop();
        SWERVE.setBrake(false);
    }
}
