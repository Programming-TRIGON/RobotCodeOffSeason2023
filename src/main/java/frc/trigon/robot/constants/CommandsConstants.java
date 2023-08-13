package frc.trigon.robot.constants;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.trigon.robot.components.XboxController;
import frc.trigon.robot.subsystems.arm.Arm;
import frc.trigon.robot.subsystems.arm.ArmConstants;
import frc.trigon.robot.subsystems.collector.Collector;
import frc.trigon.robot.subsystems.collector.CollectorConstants;
import frc.trigon.robot.subsystems.sideshooter.SideShooter;
import frc.trigon.robot.subsystems.sideshooter.SideShooterConstants;
import frc.trigon.robot.subsystems.swerve.Swerve;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;

public class CommandsConstants {
    private static final Arm ARM = Arm.getInstance();
    private static final SideShooter SIDE_SHOOTER = SideShooter.getInstance();
    private static final Collector COLLECTOR = Collector.getInstance();
    private static final XboxController DRIVER_CONTROLLER = OperatorConstants.DRIVER_CONTROLLER;

    public static final CommandBase
            GO_TO_DEFAULT_ARM_STATE_COMMAND = ARM.getGoToArmStateCommand(ArmConstants.ArmStates.DEFAULT),
            GO_TO_DEFAULT_SIDE_SHOOTER_STATE_COMMAND = SIDE_SHOOTER.getSetTargetShooterState(SideShooterConstants.SideShooterState.DEFAULT),
            STOP_COLLECTOR_COMMAND = COLLECTOR.getSetTargetStateCommand(CollectorConstants.CollectorState.STOP),
            FIELD_RELATIVE_DRIVE_COMMAND = SwerveCommands.getFieldRelativeDriveCommand(
                    () -> DRIVER_CONTROLLER.getLeftY() / OperatorConstants.STICKS_SPEED_DIVIDER / calculateShiftModeValue(),
                    () -> DRIVER_CONTROLLER.getLeftX() / OperatorConstants.STICKS_SPEED_DIVIDER / calculateShiftModeValue(),
                    () -> DRIVER_CONTROLLER.getRightX() / OperatorConstants.STICKS_SPEED_DIVIDER / calculateShiftModeValue()
            );

    private static double calculateShiftModeValue() {
        final double squaredShiftModeValue = Math.pow(DRIVER_CONTROLLER.getRightTriggerAxis(), 2);

        return 1 - squaredShiftModeValue * OperatorConstants.MINIMUM_SHIFT_VALUE_COEFFICIENT;
    }
}
