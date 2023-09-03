package frc.trigon.robot.subsystems.sideshooter;


import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.commands.Commands;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.sideshooter.simulationsideshooter.SimulationSideShooterConstants;
import frc.trigon.robot.subsystems.sideshooter.simulationsideshooter.SimulationSideShooterIO;
import frc.trigon.robot.subsystems.sideshooter.kablamasideshooter.KablamaSideShooterConstants;
import frc.trigon.robot.subsystems.sideshooter.kablamasideshooter.KablamaSideShooterIO;
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

    /**
     * Constructs a command that goes to the target state's angle, and then starts applying the state's power.
     *
     * @param sideShooterState the target side shooter state
     * @return the command
     */
    public CommandBase getSetTargetShooterStateCommand(SideShooterConstants.SideShooterState sideShooterState) {
        final Rotation2d targetAngle = sideShooterState.angle;
        final double targetPower = sideShooterState.power;

        final SequentialCommandGroup setTargetShooterStateCommand = new SequentialCommandGroup(
                getSetTargetShooterAngleCommand(targetAngle).until(() -> angleAtTarget(targetAngle)),
                Commands.withoutRequirements(getSetTargetShooterPowerCommand(targetPower))
                        .alongWith(Commands.withoutRequirements(getSetTargetShooterAngleCommand(targetAngle)))
        );

        return Commands.withRequirements(setTargetShooterStateCommand, this);
    }

    /**
     * Sets the target angle of the side collector.
     *
     * @param angle the target angle
     * @return the command
     */
    public CommandBase getSetTargetShooterAngleCommand(Rotation2d angle) {
        return new FunctionalCommand(
                () -> generateAngleMotorProfile(angle),
                this::setTargetAngleFromProfile,
                (interrupted) -> {},
                () -> false,
                this
        );
    }

    /**
     * Sets the target power to apply to the shooting motor.
     *
     * @param power the target shooter power
     * @return the command
     */
    public CommandBase getSetTargetShooterPowerCommand(double power) {
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
        Logger.getInstance().recordOutput("SideShooter/SideShooterPoses/sideShooterPose", getSideShooterPose());
        Logger.getInstance().recordOutput("SideShooter/SideShooterPoses/targetSideShooterPose", getTargetSideShooterPose());
    }

    private Pose3d getSideShooterPose() {
        return new Pose3d(
                SideShooterConstants.SIDE_SHOOTER_TRANSLATION,
                new Rotation3d(0, -Units.degreesToRadians(sideShooterInputs.angleMotorPositionDegrees), SideShooterConstants.SIDE_SHOOTER_YAW)
        );
    }

    private Pose3d getTargetSideShooterPose() {
        return new Pose3d(
                SideShooterConstants.SIDE_SHOOTER_TRANSLATION,
                new Rotation3d(0, generatedAngle.unaryMinus().getRadians(), SideShooterConstants.SIDE_SHOOTER_YAW)
        );
    }

    private SideShooterConstants getConstants() {
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.KABLAMA)
            return new KablamaSideShooterConstants();

        return new SimulationSideShooterConstants();
    }

    private SideShooterIO generateIO() {
        if (RobotConstants.IS_REPLAY)
            return new SideShooterIO();
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.KABLAMA)
            return new KablamaSideShooterIO();

        return new SimulationSideShooterIO();
    }
}

