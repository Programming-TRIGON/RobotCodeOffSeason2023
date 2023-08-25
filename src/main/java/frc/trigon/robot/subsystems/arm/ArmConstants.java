package frc.trigon.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import java.util.HashMap;

public abstract class ArmConstants {
    public static final Rotation2d DEFAULT_ARM_ANGLE = Rotation2d.fromDegrees(50);
    public static final double RETRACTED_ARM_LENGTH = 0.65;
    public static final double MAX_ARM_LENGTH = 1.7;
    public static final double ELEVATOR_METERS_PER_REVOLUTION = 0.1885;

    static final double
            ARM_ROOT_HEIGHT = 0.36,
            ARM_ROOT_X = -0.15,
            ELEVATOR_LEVELS_HEIGHT_DIFFERENCE = 0.091,
            SECOND_ELEVATOR_LEVEL_X_DIFFERENCE = -0.03,
            THIRD_ELEVATOR_LEVEL_X_DIFFERENCE = -0.022;
    static final Translation3d ARM_ROOT_TRANSLATION = new Translation3d(
            ARM_ROOT_X, 0, ARM_ROOT_HEIGHT
    );
    static final double
            THIRD_ELEVATOR_LEVEL_EXTENDED_LENGTH = 0.528,
            SECOND_ELEVATOR_LEVEL_EXTENDED_LENGTH = 0.525;

    static final double
            ELEVATOR_MOTOR_POSITION_TOLERANCE = 0.1,
            ANGLE_MOTOR_ANGLE_TOLERANCE = 1,
            ELEVATOR_MOTOR_VELOCITY_TOLERANCE = 0.1,
            ANGLE_MOTOR_VELOCITY_TOLERANCE = 10;

    private static final double
            MAX_ANGLE_VELOCITY = 100,
            MAX_ANGLE_ACCELERATION = 100,
            MAX_ELEVATOR_VELOCITY = 5,
            MAX_ELEVATOR_ACCELERATION = 5;
    static final TrapezoidProfile.Constraints
            ANGLE_CONSTRAINTS = new TrapezoidProfile.Constraints(
                    MAX_ANGLE_VELOCITY, MAX_ANGLE_ACCELERATION
            ),
            ELEVATOR_CONSTRAINTS = new TrapezoidProfile.Constraints(
                    MAX_ELEVATOR_VELOCITY, MAX_ELEVATOR_ACCELERATION
            );

    static final Mechanism2d ARM_MECHANISM = new Mechanism2d(MAX_ARM_LENGTH * 2, MAX_ARM_LENGTH, new Color8Bit(Color.kBlack));
    private static final MechanismRoot2d ARM_MECHANISM_ROOT = ARM_MECHANISM.getRoot("ArmRoot", MAX_ARM_LENGTH, 0.2);
    static final MechanismLigament2d
            TARGET_ARM_LIGAMENT = ARM_MECHANISM_ROOT.append(new MechanismLigament2d("targetArmLigament", RETRACTED_ARM_LENGTH, 0, 10, new Color8Bit(Color.kGray))),
            ARM_LIGAMENT = ARM_MECHANISM_ROOT.append(new MechanismLigament2d("zShowfirst armLigament", RETRACTED_ARM_LENGTH, 0, 10, new Color8Bit(Color.kBlue)));


    /**
     * @return the angle motor feedforward
     */
    protected abstract HashMap<Double, ArmFeedforward> getHeightToAngleMotorFeedforwardMap();

    /**
     * @return the elevator motor feedforward
     */
    protected abstract ElevatorFeedforward getElevatorMotorFeedforward();

    public enum ArmState {
        DEFAULT(0, DEFAULT_ARM_ANGLE),
        HIGH_CONE(0.5, Rotation2d.fromDegrees(45)),
        MIDDLE_CONE(0.3, Rotation2d.fromDegrees(30)),
        DOUBLE_SUBSTATION(0.2, Rotation2d.fromDegrees(46.5)),
        STANDING_CONE_COLLECTION(0, Rotation2d.fromDegrees(-10));

        ArmState(double elevatorPosition, Rotation2d angle) {
            this.elevatorPosition = elevatorPosition;
            this.angle = angle;
        }

        public final double elevatorPosition;
        public final Rotation2d angle;
    }
}
