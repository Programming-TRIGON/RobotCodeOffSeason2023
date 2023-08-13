package frc.trigon.robot.subsystems.sideshooter;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public abstract class SideShooterConstants {
    public static final double SIDE_SHOOTER_LENGTH = 0.4;
    static final Mechanism2d SIDE_SHOOTER_MECHANISM = new Mechanism2d(SIDE_SHOOTER_LENGTH * 2, SIDE_SHOOTER_LENGTH * 2, new Color8Bit(Color.kBlack));
    private static final MechanismRoot2d SIDE_SHOOTER_MECHANISM_ROOT = SIDE_SHOOTER_MECHANISM.getRoot("Side Shooter Root", SIDE_SHOOTER_LENGTH, SIDE_SHOOTER_LENGTH);
    static final MechanismLigament2d
            TARGET_SIDE_SHOOTER_LIGAMENT = SIDE_SHOOTER_MECHANISM_ROOT.append(new MechanismLigament2d("Target Side Shooter Ligament", SIDE_SHOOTER_LENGTH, 0, 10, new Color8Bit(Color.kGray))),
            SIDE_SHOOTER_LIGAMENT = SIDE_SHOOTER_MECHANISM_ROOT.append(new MechanismLigament2d("zShowfirst Side Shooter` Ligament", SIDE_SHOOTER_LENGTH, 0, 10, new Color8Bit(Color.kBlue)));

    private static final double
            MAX_ANGLE_VELOCITY = 100,
            MAX_ANGLE_ACCELERATION = 100;
    static final TrapezoidProfile.Constraints ANGLE_CONSTRAINTS = new TrapezoidProfile.Constraints(
            MAX_ANGLE_VELOCITY, MAX_ANGLE_ACCELERATION
    );

    static final double ANGLE_TOLERANCE_DEGREES = 2;

    /**
     * @return the angle motor feedforward
     */
    protected abstract ArmFeedforward getAngleMotorFeedforward();

    public enum SideShooterState {
        DEFAULT(Rotation2d.fromDegrees(0), 0),
        HIGH(Rotation2d.fromDegrees(0), 1),
        MIDDLE(Rotation2d.fromDegrees(0), 1),
        LOW(Rotation2d.fromDegrees(0), 1),
        COLLECTION(Rotation2d.fromDegrees(0), -1);

        SideShooterState(Rotation2d angle, double power) {
            this.power = power;
            this.angle = angle;
        }

        final double power;
        final Rotation2d angle;
    }
}
