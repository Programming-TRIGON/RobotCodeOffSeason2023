package frc.trigon.robot.subsystems.swerve;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.function.DoubleSupplier;

public class SwerveCommands {
    private static final Swerve SWERVE = Swerve.getInstance();

    // TODO: javadocs
    public static FunctionalCommand getFieldRelativeDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier) {
        return new InstantCommand();
    }
}
