package frc.trigon.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.AssistedCubeCollectionCommand;
import frc.trigon.robot.components.XboxController;
import frc.trigon.robot.subsystems.arm.Arm;
import frc.trigon.robot.subsystems.arm.ArmConstants;
import frc.trigon.robot.subsystems.collector.Collector;
import frc.trigon.robot.subsystems.collector.CollectorConstants;
import frc.trigon.robot.subsystems.poseestimator.PoseEstimator;
import frc.trigon.robot.subsystems.sideshooter.SideShooter;
import frc.trigon.robot.subsystems.sideshooter.SideShooterConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import frc.trigon.robot.utilities.AllianceUtilities;
import frc.trigon.robot.utilities.Maths;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

public class CommandsConstants {
    public static final AtomicInteger GRID_NUMBER = new AtomicInteger(1);
    public static final AtomicBoolean IS_LEFT_COLUMN = new AtomicBoolean(true);
    public static final AtomicBoolean IS_HIGH_LEVEL = new AtomicBoolean(true);

    private static final Arm ARM = Arm.getInstance();
    private static final SideShooter SIDE_SHOOTER = SideShooter.getInstance();
    private static final Collector COLLECTOR = Collector.getInstance();
    private static final PoseEstimator POSE_ESTIMATOR = RobotContainer.POSE_ESTIMATOR;
    private static final XboxController DRIVER_CONTROLLER = OperatorConstants.DRIVER_CONTROLLER;

    public static final CommandBase
            GO_TO_DEFAULT_ARM_STATE_COMMAND = ARM.getCurrentGoToArmPositionCommand(ArmConstants.ArmState.DEFAULT.elevatorPosition, ArmConstants.ArmState.DEFAULT.angle, 100, 100),
            GO_TO_DEFAULT_SIDE_SHOOTER_STATE_COMMAND = SIDE_SHOOTER.getSetTargetShooterStateCommand(SideShooterConstants.SideShooterState.DEFAULT, false),
            STOP_COLLECTOR_COMMAND = COLLECTOR.getSetTargetStateCommand(CollectorConstants.CollectorState.STOP),
            FIELD_RELATIVE_DRIVE_COMMAND = SwerveCommands.getOpenLoopFieldRelativeDriveCommand(
                    () -> DRIVER_CONTROLLER.getLeftY() / OperatorConstants.STICKS_SPEED_DIVIDER / calculateShiftModeValue(),
                    () -> DRIVER_CONTROLLER.getLeftX() / OperatorConstants.STICKS_SPEED_DIVIDER / calculateShiftModeValue(),
                    () -> DRIVER_CONTROLLER.getRightX() / OperatorConstants.STICKS_SPEED_DIVIDER / calculateShiftModeValue(),
                    true
            ),
            MANUAL_DRIVE_WITH_TURN_TO_COMMUNITY_COMMAND = SwerveCommands.getOpenLoopFieldRelativeDriveCommand(
                    () -> DRIVER_CONTROLLER.getLeftY() / OperatorConstants.STICKS_SPEED_DIVIDER / calculateShiftModeValue(),
                    () -> DRIVER_CONTROLLER.getLeftX() / OperatorConstants.STICKS_SPEED_DIVIDER / calculateShiftModeValue(),
                    () -> Rotation2d.fromDegrees(180),
                    true
            ),
            MANUAL_DRIVE_WITH_TURN_TO_SHOOTING_ANGLE_COMMAND = SwerveCommands.getOpenLoopFieldRelativeDriveCommand(
                    () -> DRIVER_CONTROLLER.getLeftY() / OperatorConstants.STICKS_SPEED_DIVIDER / calculateShiftModeValue(),
                    () -> DRIVER_CONTROLLER.getLeftX() / OperatorConstants.STICKS_SPEED_DIVIDER / calculateShiftModeValue(),
                    () -> Rotation2d.fromDegrees(90),
                    true
            ),
            SHOOTER_RELATIVE_DRIVE_COMMAND = SwerveCommands.getOpenLoopShooterRelativeDriveCommand(
                    () -> DRIVER_CONTROLLER.getLeftY() / OperatorConstants.STICKS_SPEED_DIVIDER / calculateShiftModeValue(),
                    () -> DRIVER_CONTROLLER.getLeftX() / OperatorConstants.STICKS_SPEED_DIVIDER / calculateShiftModeValue(),
                    () -> DRIVER_CONTROLLER.getRightX() / OperatorConstants.STICKS_SPEED_DIVIDER / calculateShiftModeValue(),
                    true
            ),
            TURN_TO_GRID_COMMAND = SwerveCommands.getOpenLoopFieldRelativeDriveCommand(
                    () -> 0,
                    () -> 0,
                    () -> Rotation2d.fromRotations(0.5),
                    false
            ),
//            ASSISTED_CUBE_COLLECTION_COMMAND = new AssistedCubeCollectionCommand(
//                    CameraConstants.CUBE_DETECTION_CAMERA::getCubeYaw,
//                    CameraConstants.CUBE_DETECTION_CAMERA::hasTargets
//            ),
            ASSISTED_CUBE_COLLECTION_FOR_SIMULATION_COMMAND = new AssistedCubeCollectionCommand(
                    CommandsConstants::calculateSimulationLimelightCubePosition,
                    CommandsConstants::doesSimulationLimelightSeeCube
            ),
            RESET_HEADING_COMMAND = new InstantCommand(() -> POSE_ESTIMATOR.resetPose(changeHeading(POSE_ESTIMATOR.getCurrentPose(), new Rotation2d()))),
            SET_GRID_NUMBER_TO_1_COMMAND = new InstantCommand(() -> GRID_NUMBER.set(AllianceUtilities.isBlueAlliance() ? 1 : 3)).ignoringDisable(true),
            SET_GRID_NUMBER_TO_2_COMMAND = new InstantCommand(() -> GRID_NUMBER.set(2)).ignoringDisable(true),
            SET_GRID_NUMBER_TO_3_COMMAND = new InstantCommand(() -> GRID_NUMBER.set(AllianceUtilities.isBlueAlliance() ? 3 : 1)).ignoringDisable(true),
            SET_COLUMN_TO_LEFT_COMMAND = new InstantCommand(() -> IS_LEFT_COLUMN.set(AllianceUtilities.isBlueAlliance())).ignoringDisable(true),
            SET_COLUMN_TO_RIGHT_COMMAND = new InstantCommand(() -> IS_LEFT_COLUMN.set(!AllianceUtilities.isBlueAlliance())).ignoringDisable(true),
            SET_LEVEL_TO_HIGH_COMMAND = new InstantCommand(() -> IS_HIGH_LEVEL.set(true)).ignoringDisable(true),
            SET_LEVEL_TO_MIDDLE_COMMAND = new InstantCommand(() -> IS_HIGH_LEVEL.set(false)).ignoringDisable(true),
            COAST_COMMAND = new InstantCommand(() -> {ARM.setNeutralMode(false); SIDE_SHOOTER.setNeutralMode(false);}).ignoringDisable(true),
            BRAKE_COMMAND = new InstantCommand(() -> {ARM.setNeutralMode(true); SIDE_SHOOTER.setNeutralMode(true);}).ignoringDisable(true);

    /**
     * @return the shift mode value to be applied to the robot's speed, from the right trigger axis
     */
    public static double calculateShiftModeValue() {
        final double squaredShiftModeValue = Math.pow(DRIVER_CONTROLLER.getRightTriggerAxis(), 2);

        return 1 - squaredShiftModeValue * OperatorConstants.MINIMUM_SHIFT_VALUE_COEFFICIENT;
    }

    private static double calculateSimulationLimelightCubePosition() {
        return calculateSimulationShooterAngleFromCube().getDegrees() / -32;
    }

    private static boolean doesSimulationLimelightSeeCube() {
        return calculateSimulationShooterAngleFromCube().getDegrees() < 32 && calculateSimulationShooterAngleFromCube().getDegrees() > -32;
    }

    private static Rotation2d calculateSimulationShooterAngleFromCube() {
        final Translation2d cubePosition = AllianceUtilities.toAlliancePose(new Pose2d(7.053130, 0.908663, new Rotation2d())).getTranslation();
        final Rotation2d
                angleBetweenCube = Maths.getAngleBetweenTranslations(POSE_ESTIMATOR.getCurrentPose().getTranslation(), cubePosition),
                robotFrontAngleFromCube = angleBetweenCube.minus(POSE_ESTIMATOR.getCurrentPose().getRotation());
        return robotFrontAngleFromCube.plus(Rotation2d.fromDegrees(-90));
    }

    private static Pose2d changeHeading(Pose2d pose2d, Rotation2d newRotation) {
        return new Pose2d(
                pose2d.getTranslation(),
                newRotation
        );
    }
}
