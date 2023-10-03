// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.trigon.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.trigon.robot.commands.Commands;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.CommandsConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.arm.Arm;
import frc.trigon.robot.subsystems.arm.ArmConstants;
import frc.trigon.robot.subsystems.collector.Collector;
import frc.trigon.robot.subsystems.leds.Leds;
import frc.trigon.robot.subsystems.poseestimator.PoseEstimator;
import frc.trigon.robot.subsystems.roller.Roller;
import frc.trigon.robot.subsystems.sideshooter.SideShooter;
import frc.trigon.robot.subsystems.sideshooter.SideShooterConstants;
import frc.trigon.robot.subsystems.swerve.AutoBalanceCommand;
import frc.trigon.robot.subsystems.swerve.Swerve;
import frc.trigon.robot.utilities.AllianceUtilities;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
    public static final LoggedDashboardChooser<String> MATCH_START_AUTO_NAME_CHOOSER = new LoggedDashboardChooser<>("autoChooser");
    public static final PoseEstimator POSE_ESTIMATOR = PoseEstimator.getInstance();
    public static final Arm ARM = Arm.getInstance();
    private final SideShooter sideShooter = SideShooter.getInstance();
    private final Collector collector = Collector.getInstance();
    private final Roller roller = Roller.getInstance();
    private final Swerve swerve = Swerve.getInstance();
    private final Leds leds = Leds.getInstance();

    public RobotContainer() {
        bindCommands();
        configureMatchStartAutoChooser();
    }

    /**
     * @return the autonomous command
     */
    public CommandBase getAutonomousCommand() {
        if (MATCH_START_AUTO_NAME_CHOOSER.get() == null)
            return null;

        return Commands.getMatchStartAuto(MATCH_START_AUTO_NAME_CHOOSER.get());
    }

    private void bindCommands() {
        bindDefaultCommands();
        bindControllerCommands();
        bindSetters();
    }

    private void bindControllerCommands() {
//        OperatorConstants.FULL_COLLECTION_FROM_LEFT_SUBSTATION_TRIGGER.whileTrue(Commands.getFullCollectionFromLeftSubstationCommand());
//        OperatorConstants.FULL_COLLECTION_FROM_RIGHT_SUBSTATION_TRIGGER.whileTrue(Commands.getFullCollectionFromRightSubstationCommand());
//        OperatorConstants.FULL_CONE_PLACING_TRIGGER.whileTrue(Commands.getFullConePlacingCommand());
//        OperatorConstants.FULL_CUBE_SHOOTING_TRIGGER.whileTrue(Commands.getFullCubeShootingCommand());

        OperatorConstants.RESET_HEADING_TRIGGER.onTrue(CommandsConstants.RESET_HEADING_COMMAND);
        OperatorConstants.PRELOAD_CURRENT_MATCH_START_AUTO_TRIGGER.onTrue(Commands.getPreloadCurrentMatchStartAutoCommand());

        OperatorConstants.TOGGLE_FIELD_AND_SHOOTER_RELATIVE_DRIVE_TRIGGER.onTrue(Commands.getToggleFieldAndShooterRelativeDriveCommand());
        OperatorConstants.MANUAL_DRIVE_WITH_TURN_TO_COMMUNITY_TRIGGER.whileTrue(CommandsConstants.MANUAL_DRIVE_WITH_TURN_TO_COMMUNITY_COMMAND);
        OperatorConstants.MANUAL_DRIVE_WITH_TURN_TO_SHOOTING_ANGLE_TRIGGER.whileTrue(CommandsConstants.MANUAL_DRIVE_WITH_TURN_TO_SHOOTING_ANGLE_COMMAND);
        OperatorConstants.DRIVE_FROM_DPAD_TRIGGER.whileTrue(CommandsConstants.SELF_RELATIVE_DRIVE_FROM_DPAD_COMMAND);

        OperatorConstants.STANDING_CONE_COLLECTION_TRIGGER.whileTrue(Commands.getStandingConeCollectionCommand());
        OperatorConstants.NON_ASSISTED_CUBE_COLLECTION_TRIGGER.whileTrue(Commands.getNonAssistedCubeCollectionCommand());
        OperatorConstants.ASSISTED_CUBE_COLLECTION_TRIGGER.whileTrue(RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.SIMULATION ? CommandsConstants.ASSISTED_CUBE_COLLECTION_FOR_SIMULATION_COMMAND : CommandsConstants.ASSISTED_CUBE_COLLECTION_COMMAND);
//
        OperatorConstants.SHOOT_CUBE_TO_HIGH_LEVEL_TRIGGER.whileTrue(Commands.getShootCubeToHighLevelCommand());
        OperatorConstants.SHOOT_CUBE_TO_MIDDLE_LEVEL_TRIGGER.whileTrue(Commands.getShootCubeToMiddleLevelCommand());
        OperatorConstants.SHOOT_CUBE_TO_HYBRID_LEVEL_TRIGGER.whileTrue(Commands.getShootCubeToHybridLevelCommand());

        OperatorConstants.PLACE_CONE_AT_HIGH_LEVEL_TRIGGER.whileTrue(Commands.getPlaceConeAtHighLevelCommand());
        OperatorConstants.PLACE_CONE_AT_MIDDLE_LEVEL_TRIGGER.whileTrue(Commands.getPlaceConeAtMiddleLevelCommand());
        OperatorConstants.PLACE_CONE_AT_HYBRID_LEVEL_TRIGGER.whileTrue(Commands.getPlaceConeAtHybridLevelCommand());

        OperatorConstants.TOGGLE_BRAKE_MODE_TRIGGER.onTrue(Commands.getToggleBrakeAndCoastCommand());
        OperatorConstants.COLLECT_FROM_SUBSTATION_TRIGGER.whileTrue(Commands.getCollectFromSubstationCommand());

        OperatorConstants.EJECT_TRIGGER.whileTrue(CommandsConstants.EJECT_COMMAND);
        OperatorConstants.COLLECT_TRIGGER.whileTrue(CommandsConstants.COLLECT_COMMAND);
        OperatorConstants.GO_TO_DASHBOARD_ARM_POSITION_TRIGGER.whileTrue(CommandsConstants.GO_TO_DASHBOARD_ARM_POSITION_COMMAND);
        OperatorConstants.SHOOT_OVER_RAMP_TRIGGER.whileTrue(sideShooter.getSetTargetShooterStateCommand(SideShooterConstants.SideShooterState.SHOOT_OVER_RAMP, true));
        OperatorConstants.OPERATOR_CONTROLLER.f().whileTrue(new AutoBalanceCommand());
    }

    private void bindDefaultCommands() {
        ARM.setDefaultState(ArmConstants.ArmState.DEFAULT);
        sideShooter.setDefaultCommand(CommandsConstants.GO_TO_DEFAULT_SIDE_SHOOTER_STATE_COMMAND);
        collector.setDefaultCommand(CommandsConstants.HOLD_COLLECTOR_COMMAND);
        roller.setDefaultCommand(roller.getFullCloseCommand());
        swerve.setDefaultCommand(CommandsConstants.FIELD_RELATIVE_DRIVE_COMMAND);
        leds.setDefaultCommand(AllianceUtilities.isBlueAlliance() ? CommandsConstants.BLUE_LIGHT_SABER_COMMAND : CommandsConstants.RED_LIGHT_SABER_COMMAND);
    }

    private void bindSetters() {
        OperatorConstants.SET_GRID_NUMBER_TO_1_TRIGGER.onTrue(CommandsConstants.SET_GRID_NUMBER_TO_1_COMMAND);
        OperatorConstants.SET_GRID_NUMBER_TO_2_TRIGGER.onTrue(CommandsConstants.SET_GRID_NUMBER_TO_2_COMMAND);
        OperatorConstants.SET_GRID_NUMBER_TO_3_TRIGGER.onTrue(CommandsConstants.SET_GRID_NUMBER_TO_3_COMMAND);

        OperatorConstants.SET_COLUMN_TO_LEFT_TRIGGER.onTrue(CommandsConstants.SET_COLUMN_TO_LEFT_COMMAND);
        OperatorConstants.SET_COLUMN_TO_RIGHT_TRIGGER.onTrue(CommandsConstants.SET_COLUMN_TO_RIGHT_COMMAND);

        OperatorConstants.SET_LEVEL_TO_HIGH_TRIGGER.onTrue(CommandsConstants.SET_LEVEL_TO_HIGH_COMMAND);
        OperatorConstants.SET_LEVEL_TO_MIDDLE_TRIGGER.onTrue(CommandsConstants.SET_LEVEL_TO_MIDDLE_COMMAND);

        OperatorConstants.YELLOW_STATIC_COLOR_TRIGGER.toggleOnTrue(CommandsConstants.YELLOW_STATIC_COLOR_COMMAND);
        OperatorConstants.PURPLE_STATIC_COLOR_TRIGGER.toggleOnTrue(CommandsConstants.PURPLE_STATIC_COLOR_COMMAND);

        OperatorConstants.INCREMENT_ANGLE_TRIGGER.onTrue(new InstantCommand(() -> ARM.setAngleAdder(ARM.getAngleAdder().plus(Rotation2d.fromDegrees(1)))));
        OperatorConstants.DECREMENT_ANGLE_TRIGGER.onFalse(new InstantCommand(() -> ARM.setAngleAdder(ARM.getAngleAdder().plus(Rotation2d.fromDegrees(-1)))));
        OperatorConstants.INCREMENT_ELEVATOR_POSITION_TRIGGER.onTrue(new InstantCommand(() -> ARM.setElevatorPositionAdder(ARM.getElevatorPositionAdder() + 0.1)));
        OperatorConstants.DECREMENT_ELEVATOR_POSITION_TRIGGER.onTrue(new InstantCommand(() -> ARM.setElevatorPositionAdder(ARM.getElevatorPositionAdder() - 0.1)));
    }

    private void configureMatchStartAutoChooser() {
        MATCH_START_AUTO_NAME_CHOOSER.addDefaultOption("None", null);

        for (String currentPathName : AutonomousConstants.AUTONOMOUS_PATHS_NAMES) {
            if (currentPathName.contains("Red") || currentPathName.contains("Blue"))
                currentPathName = currentPathName.replace("Red", "ALLIANCE").replace("Blue", "ALLIANCE");
            MATCH_START_AUTO_NAME_CHOOSER.addOption(currentPathName, currentPathName);
        }
    }
}
