package frc.trigon.robot.subsystems.swerve;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.robot.constants.CommandsConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.leds.Leds;
import frc.trigon.robot.subsystems.leds.ledcommands.StaticColorLedCommand;
import frc.trigon.robot.subsystems.poseestimator.PoseEstimator;
import frc.trigon.robot.subsystems.roller.Roller;
import frc.trigon.robot.subsystems.sideshooter.SideShooter;
import frc.trigon.robot.subsystems.sideshooter.SideShooterConstants;
import frc.trigon.robot.subsystems.swerve.Swerve;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class CubeAlignmentCommand extends CommandBase {
    private static final PIDController CUBE_ALIGNMENT_PID = new PIDController(0.001, 0, 0);
    private final Swerve swerve = Swerve.getInstance();
    private final DoubleSupplier cubePositionSupplier;
    private final BooleanSupplier hasTargetSupplier;
    private final CommandBase redLEDCommand = new StaticColorLedCommand(Color.kRed);
    private final CommandBase greenLEDCommand = new StaticColorLedCommand(Color.kGreen);

    /**
     * Constructs a new AssistedCubeCollectionCommand.
     *
     * @param cubePositionSupplier a supplier of the cube position on the camera's screen
     * @param hasTargetSupplier    a supplier of whether the camera has a target cube visible
     */
    public CubeAlignmentCommand(DoubleSupplier cubePositionSupplier, BooleanSupplier hasTargetSupplier) {
        this.cubePositionSupplier = cubePositionSupplier;
        this.hasTargetSupplier = hasTargetSupplier;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        CUBE_ALIGNMENT_PID.setSetpoint(0);
        CUBE_ALIGNMENT_PID.setTolerance(0);
        swerve.setClosedLoop(true);
        swerve.setBrake(true);
        swerve.resetRotationController();
    }

    @Override
    public void execute() {
//        if (!Roller.getInstance().isClosed() && (Roller.getInstance().getCurrentCommand() == null || !Roller.getInstance().getCurrentCommand().equals(rollerCloseCommand))) {
//            rollerCloseCommand.schedule();
//        }
//        if (OperatorConstants.NON_ASSISTED_CUBE_COLLECTION_TRIGGER.getAsBoolean()) {
//            v.schedule();
//        } else {
//            v.cancel();
//        }
        if (!hasTargetSupplier.getAsBoolean()) {
            if (!Leds.getInstance().getCurrentCommand().equals(redLEDCommand))
                redLEDCommand.schedule();
            swerve.fieldRelativeDrive(
                    OperatorConstants.DRIVER_CONTROLLER.getLeftY() / OperatorConstants.STICKS_SPEED_DIVIDER / CommandsConstants.calculateShiftModeValue(),
                    OperatorConstants.DRIVER_CONTROLLER.getLeftX() / OperatorConstants.STICKS_SPEED_DIVIDER / CommandsConstants.calculateShiftModeValue(),
                    OperatorConstants.DRIVER_CONTROLLER.getRightX() / OperatorConstants.STICKS_SPEED_DIVIDER / CommandsConstants.calculateShiftModeValue(),
                    false,
                    Rotation2d.fromDegrees(270)
            );
            swerve.resetRotationController();
            return;
        }

        if (!Leds.getInstance().getCurrentCommand().equals(greenLEDCommand))
            greenLEDCommand.schedule();
        swerve.lastAngle = PoseEstimator.getInstance().getCurrentPose().getRotation().minus(Rotation2d.fromDegrees(cubePositionSupplier.getAsDouble() + 6));
        swerve.fieldRelativeDrive(
                OperatorConstants.DRIVER_CONTROLLER.getLeftY() / OperatorConstants.STICKS_SPEED_DIVIDER / CommandsConstants.calculateShiftModeValue(),
                OperatorConstants.DRIVER_CONTROLLER.getLeftX() / OperatorConstants.STICKS_SPEED_DIVIDER / CommandsConstants.calculateShiftModeValue(),
                swerve.lastAngle,
                false,
                Rotation2d.fromDegrees(270)
        );
    }

    @Override
    public void end(boolean interrupted) {
        Leds.getInstance().getCurrentCommand().cancel();
    }

    private double calculatePIDOutput(double cubePosition, boolean hasTarget) {
        if (!hasTarget)
            return OperatorConstants.DRIVER_CONTROLLER.getRightX() / OperatorConstants.STICKS_SPEED_DIVIDER / CommandsConstants.calculateShiftModeValue();
        return CUBE_ALIGNMENT_PID.calculate(cubePosition);

    }
}