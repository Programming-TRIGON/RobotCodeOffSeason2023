package frc.trigon.robot.constants;

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
}
