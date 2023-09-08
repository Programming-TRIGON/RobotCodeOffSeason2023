package frc.trigon.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.CommandsConstants;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.arm.Arm;
import frc.trigon.robot.subsystems.arm.ArmConstants;
import frc.trigon.robot.subsystems.collector.Collector;
import frc.trigon.robot.subsystems.collector.CollectorConstants;
import frc.trigon.robot.subsystems.poseestimator.PoseEstimator;
import frc.trigon.robot.subsystems.roller.Roller;
import frc.trigon.robot.subsystems.sideshooter.SideShooter;
import frc.trigon.robot.subsystems.sideshooter.SideShooterConstants;
import frc.trigon.robot.subsystems.swerve.Swerve;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import frc.trigon.robot.utilities.AllianceUtilities;

import java.util.List;

public class Commands {
    private static final Arm ARM = Arm.getInstance();
    private static final Collector COLLECTOR = Collector.getInstance();
    private static final Roller ROLLER = Roller.getInstance();
    private static final SideShooter SIDE_SHOOTER = SideShooter.getInstance();
    private static final PoseEstimator POSE_ESTIMATOR = RobotContainer.POSE_ESTIMATOR;
    private static final Swerve SWERVE = Swerve.getInstance();

    /**
     * Generates the match start autonomous path command.
     *
     * @param pathName the path's name
     * @return the command
     */
    public static CommandBase getMatchStartAuto(String pathName) {
        final List<PathPlannerTrajectory> autonomousPathGroup;
        if (pathName.contains("ALLIANCE"))
            pathName = pathName.replace("ALLIANCE", DriverStation.getAlliance().name());

        if (AutonomousConstants.PRELOADED_PATHS.containsKey(pathName)) {
            autonomousPathGroup = AutonomousConstants.PRELOADED_PATHS.get(pathName);
        } else {
            autonomousPathGroup = PathPlanner.loadPathGroup(pathName, AutonomousConstants.AUTONOMOUS_PATH_CONSTRAINTS);
            AutonomousConstants.PRELOADED_PATHS.put(pathName, autonomousPathGroup);
        }

        return SwerveCommands.getFollowPathGroupCommand(autonomousPathGroup, AutonomousConstants.EVENT_MAP, true);
    }

    /**
     * @return a command that preloads the current chosen autonomous path
     */
    public static CommandBase getPreloadCurrentMatchStartAutoCommand() {
        return new InstantCommand(() -> {
            String pathName = RobotContainer.MATCH_START_AUTO_NAME_CHOOSER.get();
            if (pathName == null)
                return;
            if (pathName.contains("ALLIANCE"))
                pathName = pathName.replace("ALLIANCE", DriverStation.getAlliance().name());

            final List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathName, AutonomousConstants.AUTONOMOUS_PATH_CONSTRAINTS);
            final Pose2d initialPose = pathGroup.get(0).getInitialHolonomicPose();
            final Pose2d initialAlliancePose = new Pose2d(
                    initialPose.getX(),
                    AllianceUtilities.isBlueAlliance() ? initialPose.getY() : FieldConstants.FIELD_WIDTH_METERS - initialPose.getY(),
                    initialPose.getRotation()
            );

            POSE_ESTIMATOR.resetPose(initialAlliancePose);
            AutonomousConstants.PRELOADED_PATHS.put(pathName, pathGroup);
        }).ignoringDisable(true);
    }

    /**
     * Decorates a Command without its requirements.
     *
     * @param command the command to decorate
     * @return the decorated Command
     */
    public static CommandBase withoutRequirements(CommandBase command) {
        return new FunctionalCommand(
                command::initialize,
                command::execute,
                command::end,
                command::isFinished
        );
    }

    /**
     * Decorates a Command with requirements.
     *
     * @param command    the command to decorate
     * @param subsystems the subsystems to add to the requirements
     * @return the decorated Command
     */
    public static CommandBase withRequirements(CommandBase command, Subsystem... subsystems) {
        command.addRequirements(subsystems);
        return command;
    }

    /**
     * Constructs a new command that requires the given subsystems, so no other commands can use them.
     *
     * @param subsystems the subsystems to require
     * @return the command
     */
    public static CommandBase getArbitrarySubsystemCommand(Subsystem... subsystems) {
        return new StartEndCommand(
                () -> {
                },
                () -> {
                },
                subsystems
        );
    }

    /**
     * @return a command that fully places a cone, with all the cone placing logic
     */
    public static CommandBase getFullConePlacingCommand() {
        return new ProxyCommand(() -> {
            final ArmConstants.ArmState targetState = CommandsConstants.IS_HIGH_LEVEL.get() ? ArmConstants.ArmState.HIGH_CONE : ArmConstants.ArmState.MIDDLE_CONE;
            final FieldConstants.GridAlignment targetGridAlignment = FieldConstants.GridAlignment.getGridAlignment(
                    CommandsConstants.GRID_NUMBER.get(), CommandsConstants.IS_LEFT_COLUMN.get() ? 1 : 3
            );
            final CommandBase driveToGridAlignmentCommand = SwerveCommands.getEnterCommunityCommand(
                    AutonomousConstants.DRIVE_TO_GRID_ALIGNMENT_CONSTRAINTS,
                    targetGridAlignment.inFrontOfGridPose,
                    0.1,
                    5
            );

            return new SequentialCommandGroup(
                    driveToGridAlignmentCommand,
                    getWaitForContinueCommand(),
                    COLLECTOR.getSetTargetStateCommand(CollectorConstants.CollectorState.EJECT)
            ).alongWith(
                    ARM.getGoToArmStateCommand(targetState, 100, 100)
            );
        });
    }

    /**
     * @return a command that places a cone at the high level
     */
    public static CommandBase getPlaceConeAtHighLevelCommand() {
        return new SequentialCommandGroup(
                getWaitForContinueCommand(),
                COLLECTOR.getSetTargetStateCommand(CollectorConstants.CollectorState.EJECT)
        ).alongWith(
                ARM.getGoToArmStateCommand(ArmConstants.ArmState.HIGH_CONE, 100, 100)
        );
    }

    /**
     * @return a command that places a cone at the middle level
     */
    public static CommandBase getPlaceConeAtMiddleLevelCommand() {
        return new SequentialCommandGroup(
                getWaitForContinueCommand(),
                COLLECTOR.getSetTargetStateCommand(CollectorConstants.CollectorState.EJECT)
        ).alongWith(
                ARM.getGoToArmStateCommand(ArmConstants.ArmState.MIDDLE_CONE, 100, 100)
        );
    }

    /**
     * @return a command that places a cone at the hybrid level
     */
    public static CommandBase getPlaceConeAtHybridLevelCommand() {
        return new SequentialCommandGroup(
                getWaitForContinueCommand(),
                COLLECTOR.getSetTargetStateCommand(CollectorConstants.CollectorState.EJECT)
        ).alongWith(
                ARM.getGoToArmStateCommand(ArmConstants.ArmState.HYBRID_CONE, 100, 100)
        );
    }

    /**
     * @return a command that fully shoots a cube, with all the cube shooting logic
     */
    public static CommandBase getFullCubeShootingCommand() {
        return new ProxyCommand(() -> {
            final SideShooterConstants.SideShooterState targetState = CommandsConstants.IS_HIGH_LEVEL.get() ? SideShooterConstants.SideShooterState.HIGH : SideShooterConstants.SideShooterState.MIDDLE;
            final FieldConstants.GridAlignment targetGridAlignment = FieldConstants.GridAlignment.getGridAlignment(
                    CommandsConstants.GRID_NUMBER.get(), 2
            );
            final Pose2d targetPose = changeRotation(targetGridAlignment.inFrontOfGridPose, Rotation2d.fromRotations(0.25));
            final CommandBase driveToGridAlignmentCommand = SwerveCommands.getEnterCommunityCommand(
                    AutonomousConstants.DRIVE_TO_GRID_ALIGNMENT_CONSTRAINTS,
                    targetPose,
                    0.1,
                    5
            );

            return new SequentialCommandGroup(
                    ROLLER.getFullCloseCommand().until(ROLLER::isClosed),
                    driveToGridAlignmentCommand.raceWith(SIDE_SHOOTER.getSetTargetShooterAngleCommand(targetState.angle)),
                    getWaitForContinueCommand().alongWith(SIDE_SHOOTER.getSetTargetShooterAngleCommand(targetState.angle)),
                    SIDE_SHOOTER.getSetTargetShooterStateCommand(targetState).alongWith(getArbitrarySubsystemCommand(SWERVE).asProxy())
            );
        });
    }

    /**
     * @return a command that shoots a cube to the high level
     */
    public static CommandBase getShootCubeToHighLevelCommand() {
        return new SequentialCommandGroup(
                SIDE_SHOOTER.getSetTargetShooterAngleCommand(SideShooterConstants.SideShooterState.HIGH.angle).raceWith(getWaitForContinueCommand()),
                SIDE_SHOOTER.getSetTargetShooterStateCommand(SideShooterConstants.SideShooterState.HIGH)
        );
    }

    /**
     * @return a command that shoots a cube to the middle level
     */
    public static CommandBase getShootCubeToMiddleLevelCommand() {
        return new SequentialCommandGroup(
                SIDE_SHOOTER.getSetTargetShooterAngleCommand(SideShooterConstants.SideShooterState.MIDDLE.angle).raceWith(getWaitForContinueCommand()),
                SIDE_SHOOTER.getSetTargetShooterStateCommand(SideShooterConstants.SideShooterState.MIDDLE)
        );
    }

    /**
     * @return a command that shoots a cube to the hybrid level
     */
    public static CommandBase getShootCubeToHybridLevelCommand() {
        return new SequentialCommandGroup(
                SIDE_SHOOTER.getSetTargetShooterAngleCommand(SideShooterConstants.SideShooterState.HYBRID.angle).raceWith(getWaitForContinueCommand()),
                SIDE_SHOOTER.getSetTargetShooterStateCommand(SideShooterConstants.SideShooterState.HYBRID)
        );
    }

    /**
     * @return a command that fully collects from the left substation, with all the substation collection logic
     */
    public static CommandBase getFullCollectionFromLeftSubstationCommand() {
        return new ProxyCommand(() -> {
            final double substationY = AllianceUtilities.isBlueAlliance() ? FieldConstants.OUTER_SUBSTATION_Y : FieldConstants.INNER_SUBSTATION_Y;
            return getFullCollectionFromDoubleSubstationCommand(substationY);
        });
    }

    /**
     * @return a command that fully collects from the right substation, with all the substation collection logic
     */
    public static CommandBase getFullCollectionFromRightSubstationCommand() {
        return new ProxyCommand(() -> {
            final double substationY = AllianceUtilities.isBlueAlliance() ? FieldConstants.INNER_SUBSTATION_Y : FieldConstants.OUTER_SUBSTATION_Y;
            return getFullCollectionFromDoubleSubstationCommand(substationY);
        });
    }

    /**
     * @return a command that collects a standing cone
     */
    public static CommandBase getStandingConeCollectionCommand() {
        return new ParallelCommandGroup(
                ARM.getGoToArmStateCommand(ArmConstants.ArmState.STANDING_CONE_COLLECTION),
                COLLECTOR.getSetTargetStateCommand(CollectorConstants.CollectorState.COLLECT)
        );
    }

    /**
     * @return a command that shoots a cube over the charging station
     */
    public static CommandBase getShootCubeOverChargingStationCommand() {
        return new SequentialCommandGroup(
                CommandsConstants.TURN_TO_GRID_COMMAND.until(() -> SWERVE.atAngle(Rotation2d.fromRotations(0.5))),
                SIDE_SHOOTER.getSetTargetShooterStateCommand(SideShooterConstants.SideShooterState.SHOOT_OVER_RAMP)
        );
    }

    /**
     * @return a command to collect a cube from the ground, without any drive assistance
     */
    public static CommandBase getNonAssistedCubeCollectionCommand() {
        return new ParallelCommandGroup(
                ROLLER.getFullCollectionCommand(),
                SIDE_SHOOTER.getSetTargetShooterStateCommand(SideShooterConstants.SideShooterState.COLLECTION)
        );
    }

    /**
     * @return a command that toggles between the swerve's default command, from field relative to shooter relative
     */
    public static CommandBase getToggleFieldAndShooterRelativeDriveCommand() {
        return new InstantCommand(() -> {
            if (SWERVE.getDefaultCommand().equals(CommandsConstants.FIELD_RELATIVE_DRIVE_COMMAND))
                SWERVE.setDefaultCommand(CommandsConstants.SHOOTER_RELATIVE_DRIVE_COMMAND);
            else
                SWERVE.setDefaultCommand(CommandsConstants.FIELD_RELATIVE_DRIVE_COMMAND);

            SWERVE.getDefaultCommand().schedule();
        });
    }

    /**
     * @return a command that fully closes the roller and the side shooter
     */
    public static CommandBase getFullCollectionCloseCommand() {
        return new ParallelCommandGroup(
                ROLLER.getFullCloseCommand(),
                SIDE_SHOOTER.getSetTargetShooterStateCommand(SideShooterConstants.SideShooterState.DEFAULT)
        );
    }

    /**
     * @return a command that places a cone at the high node, for autonomous
     */
    public static CommandBase getPlaceConeAtHighForAutoCommand() {
        return new SequentialCommandGroup(
                ARM.getGoToArmStateCommand(ArmConstants.ArmState.HIGH_CONE).until(() -> ARM.atState(ArmConstants.ArmState.HIGH_CONE)),
                COLLECTOR.getSetTargetStateCommand(CollectorConstants.CollectorState.EJECT).alongWith(ARM.getGoToArmStateCommand(ArmConstants.ArmState.HIGH_CONE))
        );
    }

    private static CommandBase getFullCollectionFromDoubleSubstationCommand(double substationY) {
        final CommandBase xAxisDriveCommand = SwerveCommands.getClosedLoopXAxisDriveCommand(
                () -> OperatorConstants.DRIVER_CONTROLLER.getLeftY() / OperatorConstants.STICKS_SPEED_DIVIDER / CommandsConstants.calculateShiftModeValue(),
                substationY,
                new Rotation2d(),
                true
        );

        return new ParallelCommandGroup(
                xAxisDriveCommand,
                COLLECTOR.getSetTargetStateCommand(CollectorConstants.CollectorState.COLLECT),
                ARM.getGoToArmStateCommand(ArmConstants.ArmState.DOUBLE_SUBSTATION, 100, 100)
        );
    }

    private static CommandBase getWaitForContinueCommand() {
        return new WaitUntilCommand(OperatorConstants.CONTINUE_TRIGGER);
    }

    private static Pose2d changeRotation(Pose2d pose2d, Rotation2d newRotation) {
        return new Pose2d(
                pose2d.getX(),
                pose2d.getY(),
                newRotation
        );
    }
}
