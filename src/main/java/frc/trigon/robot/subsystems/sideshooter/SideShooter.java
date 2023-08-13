package frc.trigon.robot.subsystems.sideshooter;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.sideshooter.simulationsideshooter.SimulationSideShooterConstants;
import frc.trigon.robot.subsystems.sideshooter.simulationsideshooter.SimulationSideShooterIO;
import frc.trigon.robot.subsystems.sideshooter.staticsideshooter.StaticSideShooterConstants;
import frc.trigon.robot.subsystems.sideshooter.staticsideshooter.StaticSideShooterIO;
import org.littletonrobotics.junction.Logger;

public class SideShooter extends SubsystemBase {
    private final static SideShooter INSTANCE = new SideShooter();
    private final SideShooterIO sideShooterIO;
    private final SideShooterInputsAutoLogged sideShooterInputs = new SideShooterInputsAutoLogged();
    private final SideShooterConstants sideShooterConstants;

    private TrapezoidProfile angleMotorProfile = null;
    private double lastAngleMotorGenerationProfileTime;
    private Rotation2d generatedAngle = new Rotation2d();

    public static SideShooter getInstance() {
        return INSTANCE;
    }

    private SideShooter() {
        sideShooterIO = generateIO();
        sideShooterConstants = getConstants();
    }

    @Override
    public void periodic() {
        sideShooterIO.updateInputs(sideShooterInputs);
        Logger.getInstance().processInputs("SideShooter", sideShooterInputs);
        updateMechanism();
    }

    // TODO: javadocs
    public CommandBase getSetTargetShooterState(SideShooterConstants.SideShooterState sideShooterState) {
        final Rotation2d targetAngle = sideShooterState.angle;
        final double targetVelocity = sideShooterState.power;

        return new ParallelCommandGroup(
                getSetTargetShooterAngle(targetAngle).until(() -> angleAtTarget(targetAngle)),
                getSetTargetShooterPower(targetVelocity)
        );
    }

    public CommandBase getSetTargetShooterAngle(Rotation2d angle) {
        return new FunctionalCommand(
                () -> generateAngleMotorProfile(angle),
                this::setTargetAngleFromProfile,
                (interrupted) -> {},
                () -> false,
                this
        );
    }

    public CommandBase getSetTargetShooterPower(double power) {
        return new StartEndCommand(
                () -> sideShooterIO.setTargetShootingPower(power),
                () -> {},
                this
        );
    }

    private void generateAngleMotorProfile(Rotation2d targetAngle) {
        angleMotorProfile = new TrapezoidProfile(
                SideShooterConstants.ANGLE_CONSTRAINTS,
                new TrapezoidProfile.State(targetAngle.getDegrees(), 0),
                new TrapezoidProfile.State(sideShooterInputs.angleMotorPositionDegrees, sideShooterInputs.angleMotorVelocityDegreesPerSecond)
        );

        lastAngleMotorGenerationProfileTime = Timer.getFPGATimestamp();
    }

    private void setTargetAngleFromProfile() {
        if (angleMotorProfile == null) {
            sideShooterIO.stopAngleMotor();
            return;
        }

        final TrapezoidProfile.State targetState = angleMotorProfile.calculate(getAngleMotorProfileTime());
        final double feedforward = sideShooterConstants.getAngleMotorFeedforward().calculate(targetState.position, targetState.velocity);
        final Rotation2d targetAngle = Rotation2d.fromDegrees(targetState.position);

        sideShooterIO.setTargetAngle(targetAngle, feedforward);
        generatedAngle = targetAngle;
    }

    private double getAngleMotorProfileTime() {
        return Timer.getFPGATimestamp() - lastAngleMotorGenerationProfileTime;
    }

    private boolean angleAtTarget(Rotation2d targetAngle) {
        return Math.abs(targetAngle.getDegrees() - sideShooterInputs.angleMotorPositionDegrees) < SideShooterConstants.ANGLE_TOLERANCE_DEGREES;
    }

    private void updateMechanism() {
        SideShooterConstants.TARGET_SIDE_SHOOTER_LIGAMENT.setAngle(generatedAngle.getDegrees());
        SideShooterConstants.SIDE_SHOOTER_LIGAMENT.setAngle(sideShooterInputs.angleMotorPositionDegrees);

        Logger.getInstance().recordOutput("SideShooter/SideShooterMechanism", SideShooterConstants.SIDE_SHOOTER_MECHANISM);
    }

    private SideShooterConstants getConstants() {
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.STATIC)
            return new StaticSideShooterConstants();

        return new SimulationSideShooterConstants();
    }

    private SideShooterIO generateIO() {
        if (RobotConstants.IS_REPLAY)
            return new SideShooterIO();
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.STATIC)
            return new StaticSideShooterIO();

        return new SimulationSideShooterIO();
    }
}

