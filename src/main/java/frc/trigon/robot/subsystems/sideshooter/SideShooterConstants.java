package frc.trigon.robot.subsystems.sideshooter;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public abstract class SideShooterConstants {
    public static final double SIDE_SHOOTER_LENGTH = 0.4;
    static final Mechanism2d SIDE_SHOOTER_MECHANISM = new Mechanism2d(SIDE_SHOOTER_LENGTH * 2, SIDE_SHOOTER_LENGTH, new Color8Bit(Color.kBlack));
    private static final MechanismRoot2d SIDE_SHOOTER_MECHANISM_ROOT = SIDE_SHOOTER_MECHANISM.getRoot("SideShooterRoot", 0.4, 0.2);
    static final MechanismLigament2d
            TARGET_SIDE_SHOOTER_LIGAMENT = SIDE_SHOOTER_MECHANISM_ROOT.append(new MechanismLigament2d("targetSideShooterLigament", SIDE_SHOOTER_LENGTH, 0, 10, new Color8Bit(Color.kGray))),
            SIDE_SHOOTER_LIGAMENT = SIDE_SHOOTER_MECHANISM_ROOT.append(new MechanismLigament2d("zShowfirst sideShooterLigament", SIDE_SHOOTER_LENGTH, 0, 10, new Color8Bit(Color.kBlue)));
    static final double SIDE_SHOOTER_YAW = Units.degreesToRadians(90);
    private static final double
            SIDE_SHOOTER_X = 0,
            SIDE_SHOOTER_Y = 0.26,
            SIDE_SHOOTER_Z = 0.135;
    static final Translation3d SIDE_SHOOTER_TRANSLATION = new Translation3d(
            SIDE_SHOOTER_X, SIDE_SHOOTER_Y, SIDE_SHOOTER_Z
    );

    private static final double
            MAX_ANGLE_VELOCITY = 600,
            MAX_ANGLE_ACCELERATION = 500;
    static final TrapezoidProfile.Constraints ANGLE_CONSTRAINTS = new TrapezoidProfile.Constraints(
            MAX_ANGLE_VELOCITY, MAX_ANGLE_ACCELERATION
    );
    static final double ANGLE_TOLERANCE_DEGREES = 2;

    static final double COLLECTION_CURRENT_THRESHOLD = 25;
    static final double COLLECTION_TIME_THRESHOLD = 0.2;

    /**
     * @return the angle motor feedforward
     */
    protected abstract ArmFeedforward getAngleMotorFeedforward();

    public enum SideShooterState {
        DEFAULT(Rotation2d.fromDegrees(90), 0),
        HIGH(Rotation2d.fromDegrees(53), 5.3),
        MIDDLE(Rotation2d.fromDegrees(60), 4.1),
        HYBRID(Rotation2d.fromDegrees(10), 4),
        HIGH_FROM_AFAR(Rotation2d.fromDegrees(45), 16),
        MIDDLE_FROM_AFAR(Rotation2d.fromDegrees(30), 13),
        HYBRID_FROM_AFAR(Rotation2d.fromDegrees(20), 12),
        COLLECTION(Rotation2d.fromDegrees(0), -3.5),
        SHOOT_OVER_RAMP(Rotation2d.fromDegrees(40), 16);

        public final Rotation2d angle;
        public final double voltage;

        SideShooterState(Rotation2d angle, double voltage) {
            this.angle = angle;
            this.voltage = voltage;
        }
    }
}
