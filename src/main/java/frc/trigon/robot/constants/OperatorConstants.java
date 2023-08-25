package frc.trigon.robot.constants;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.trigon.robot.components.KeyboardController;
import frc.trigon.robot.components.XboxController;

public class OperatorConstants {
    private static final int
            DRIVER_CONTROLLER_PORT = 0,
            OPERATOR_CONTROLLER_PORT = 1;
    private static final int DRIVER_CONTROLLER_EXPONENT = 2;
    private static final double DRIVER_CONTROLLER_DEADBAND = 0.1;
    public static final XboxController DRIVER_CONTROLLER = new XboxController(
            DRIVER_CONTROLLER_PORT, DRIVER_CONTROLLER_EXPONENT, DRIVER_CONTROLLER_DEADBAND
    );
    public static final KeyboardController OPERATOR_CONTROLLER = new KeyboardController(OPERATOR_CONTROLLER_PORT);

    public static final double STICKS_SPEED_DIVIDER = 1;
    private static final double MINIMUM_SHIFT_VALUE = 0.18;
    public static final double MINIMUM_SHIFT_VALUE_COEFFICIENT = 1 - (1 / MINIMUM_SHIFT_VALUE);

    public static final Trigger
            SET_GRID_NUMBER_TO_1_TRIGGER = OPERATOR_CONTROLLER.numpad7(),
            SET_GRID_NUMBER_TO_2_TRIGGER = OPERATOR_CONTROLLER.numpad8(),
            SET_GRID_NUMBER_TO_3_TRIGGER = OPERATOR_CONTROLLER.numpad9(),
            SET_COLUMN_TO_LEFT_TRIGGER = OPERATOR_CONTROLLER.numpad4(),
            SET_COLUMN_TO_RIGHT_TRIGGER = OPERATOR_CONTROLLER.numpad6(),
            SET_LEVEL_TO_HIGH_TRIGGER = OPERATOR_CONTROLLER.numpad5(),
            SET_LEVEL_TO_MIDDLE_TRIGGER = OPERATOR_CONTROLLER.numpad2(),
            CONTINUE_TRIGGER = OPERATOR_CONTROLLER.k().or(DRIVER_CONTROLLER.leftBumper()),
            RESET_HEADING_TRIGGER = DRIVER_CONTROLLER.y(),
            PRELOAD_CURRENT_MATCH_START_AUTO_TRIGGER = OPERATOR_CONTROLLER.p(),
            TOGGLE_FIELD_AND_SELF_RELATIVE_DRIVE_TRIGGER = DRIVER_CONTROLLER.b(),
            MANUAL_DRIVE_WITH_TURN_TO_COMMUNITY_TRIGGER = DRIVER_CONTROLLER.x(),
            MANUAL_DRIVE_WITH_TURN_TO_SHOOTING_ANGLE_TRIGGER = DRIVER_CONTROLLER.a(),
            STANDING_CONE_COLLECTION_TRIGGER = DRIVER_CONTROLLER.rightBumper(),
            NON_ASSISTED_CUBE_COLLECTION_TRIGGER = DRIVER_CONTROLLER.leftTrigger(),
            ASSISTED_CUBE_COLLECTION_TRIGGER = OPERATOR_CONTROLLER.a(),
            FULL_COLLECTION_FROM_LEFT_SUBSTATION_TRIGGER = OPERATOR_CONTROLLER.x(),
            FULL_COLLECTION_FROM_RIGHT_SUBSTATION_TRIGGER = OPERATOR_CONTROLLER.c(),
            FULL_CONE_PLACING_TRIGGER = OPERATOR_CONTROLLER.b(),
            FULL_CUBE_SHOOTING_TRIGGER = OPERATOR_CONTROLLER.v();
}
