package frc.trigon.robot.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.trigon.robot.constants.CommandsConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.roller.Roller;
import frc.trigon.robot.subsystems.sideshooter.SideShooter;
import frc.trigon.robot.subsystems.sideshooter.SideShooterConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class AssistedCubeCollectionCommand extends ParallelCommandGroup {
    private static final PIDController CUBE_ALIGNMENT_PID = new PIDController(0.8, 0, 0);
    private double lastPower = Double.NaN;

    public AssistedCubeCollectionCommand(DoubleSupplier cubePositionSupplier, BooleanSupplier hasTargetSupplier) {
        super();
        CUBE_ALIGNMENT_PID.setSetpoint(0);
        final CommandBase manualDriveWhileLockingOnCubeCommand = SwerveCommands.getClosedLoopFieldRelativeDriveCommand(
                () -> OperatorConstants.DRIVER_CONTROLLER.getLeftY() / OperatorConstants.STICKS_SPEED_DIVIDER / CommandsConstants.calculateShiftModeValue(),
                () -> OperatorConstants.DRIVER_CONTROLLER.getLeftX() / OperatorConstants.STICKS_SPEED_DIVIDER / CommandsConstants.calculateShiftModeValue(),
                () -> calculatePIDOutput(cubePositionSupplier.getAsDouble(), hasTargetSupplier.getAsBoolean()),
                true
        );
        super.addCommands(
                manualDriveWhileLockingOnCubeCommand,
                Roller.getInstance().getFullCollectionCommand(),
                SideShooter.getInstance().getSetTargetShooterState(SideShooterConstants.SideShooterState.COLLECTION)
        );
    }

    private double calculatePIDOutput(double cubePosition, boolean hasTarget) {
        if (!hasTarget && !Double.isNaN(lastPower))
            return Math.signum(lastPower) * 0.4;
        if (hasTarget) {
            lastPower = CUBE_ALIGNMENT_PID.calculate(cubePosition);
            return lastPower;
        }
        return 0.4;
    }
}