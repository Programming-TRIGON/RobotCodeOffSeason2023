package frc.trigon.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.subsystems.swerve.CubeAlignmentCommand;
import frc.trigon.robot.components.XboxController;
import frc.trigon.robot.subsystems.arm.Arm;
import frc.trigon.robot.subsystems.collector.Collector;
import frc.trigon.robot.subsystems.collector.CollectorConstants;
import frc.trigon.robot.subsystems.leds.ledcommands.MovingColorLedCommand;
import frc.trigon.robot.subsystems.leds.ledcommands.StaticColorLedCommand;
import frc.trigon.robot.subsystems.poseestimator.PoseEstimator;
import frc.trigon.robot.subsystems.sideshooter.SideShooter;
import frc.trigon.robot.subsystems.sideshooter.SideShooterConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import frc.trigon.robot.utilities.AllianceUtilities;
import frc.trigon.robot.utilities.Maths;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

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

    private static final LoggedDashboardNumber
            ELEVATOR_POSITION_DASHBOARD_NUMBER = new LoggedDashboardNumber("armElevatorPosition"),
            ANGLE_DEGREES_DASHBOARD_NUMBER = new LoggedDashboardNumber("armAngleDegrees");

    public static final CommandBase
            GO_TO_DEFAULT_SIDE_SHOOTER_STATE_COMMAND = SIDE_SHOOTER.getSetTargetShooterStateCommand(SideShooterConstants.SideShooterState.DEFAULT, false),
            HOLD_COLLECTOR_COMMAND = COLLECTOR.getSetTargetStateCommand(CollectorConstants.CollectorState.HOLD),
            FIELD_RELATIVE_DRIVE_COMMAND = SwerveCommands.getClosedLoopFieldRelativeDriveCommand(
                    () -> DRIVER_CONTROLLER.getLeftY() / OperatorConstants.STICKS_SPEED_DIVIDER / calculateShiftModeValue(),
                    () -> DRIVER_CONTROLLER.getLeftX() / OperatorConstants.STICKS_SPEED_DIVIDER / calculateShiftModeValue(),
                    () -> DRIVER_CONTROLLER.getRightX() / OperatorConstants.STICKS_SPEED_DIVIDER / calculateShiftModeValue(),
                    true
            ),
            SHOOTER_RELATIVE_DRIVE_COMMAND = SwerveCommands.getOpenLoopShooterRelativeDriveCommand(
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
            TURN_TO_GRID_COMMAND = SwerveCommands.getOpenLoopFieldRelativeDriveCommand(
                    () -> 0,
                    () -> 0,
                    () -> Rotation2d.fromRotations(0.5),
                    true
            ),
            ASSISTED_CUBE_COLLECTION_COMMAND = new CubeAlignmentCommand(
                    CameraConstants.CUBE_DETECTION_CAMERA::getCubeYaw,
                    CameraConstants.CUBE_DETECTION_CAMERA::hasTargets
            ),
            ASSISTED_CUBE_COLLECTION_FOR_SIMULATION_COMMAND = new CubeAlignmentCommand(
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
            YELLOW_STATIC_COLOR_COMMAND = new StaticColorLedCommand(Color.kYellow),
            PURPLE_STATIC_COLOR_COMMAND = new StaticColorLedCommand(Color.kPurple),
            BLUE_LIGHT_SABER_COMMAND = new MovingColorLedCommand(Color.kBlue, new Color(0, 0, Color.kBlue.blue * 0.2), 0.02, 10),
            RED_LIGHT_SABER_COMMAND = new MovingColorLedCommand(Color.kRed, new Color(Color.kRed.red * 0.2, 0, 0), 0.02, 10),
            GO_TO_DASHBOARD_ARM_POSITION_COMMAND = new ProxyCommand(() -> ARM.getGoToArmPositionCommand(ELEVATOR_POSITION_DASHBOARD_NUMBER.get(), Rotation2d.fromDegrees(ANGLE_DEGREES_DASHBOARD_NUMBER.get()))),
            EJECT_COMMAND = COLLECTOR.getSetTargetStateCommand(CollectorConstants.CollectorState.EJECT),
            COLLECT_COMMAND = COLLECTOR.getSetTargetStateCommand(CollectorConstants.CollectorState.COLLECT),
            SELF_RELATIVE_DRIVE_FROM_DPAD_COMMAND = SwerveCommands.getOpenLoopSelfRelativeDriveCommand(
                    () -> Math.cos(Units.degreesToRadians(DRIVER_CONTROLLER.getPov())) / OperatorConstants.POV_DIVIDER / calculateShiftModeValue(),
                    () -> Math.sin(Units.degreesToRadians(-DRIVER_CONTROLLER.getPov())) / OperatorConstants.POV_DIVIDER / calculateShiftModeValue(),
                    () -> 0,
                    true
            ),
            STATIC_WHITE_COLOR_COMMAND = new StaticColorLedCommand(Color.kWhite);

    /**
     * @return the shift mode value to be applied to the robot's speed, from the right trigger axis
     */
    public static double calculateShiftModeValue() {
        final double shiftModeValue = DRIVER_CONTROLLER.getRightTriggerAxis();

        return 1 - shiftModeValue * OperatorConstants.MINIMUM_SHIFT_VALUE_COEFFICIENT;
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
