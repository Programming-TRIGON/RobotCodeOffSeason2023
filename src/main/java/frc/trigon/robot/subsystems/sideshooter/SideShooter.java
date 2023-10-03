package frc.trigon.robot.subsystems.sideshooter;


import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.Robot;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.Commands;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.arm.ArmConstants;
import frc.trigon.robot.subsystems.sideshooter.kablamasideshooter.KablamaSideShooterConstants;
import frc.trigon.robot.subsystems.sideshooter.kablamasideshooter.KablamaSideShooterIO;
import frc.trigon.robot.subsystems.sideshooter.simulationsideshooter.SimulationSideShooterConstants;
import frc.trigon.robot.subsystems.sideshooter.simulationsideshooter.SimulationSideShooterIO;
import frc.trigon.robot.utilities.CurrentWatcher;
import org.littletonrobotics.junction.Logger;

public class SideShooter extends SubsystemBase {
    private final static SideShooter INSTANCE = new SideShooter();
    private final SideShooterIO sideShooterIO;
    private final SideShooterInputsAutoLogged sideShooterInputs = new SideShooterInputsAutoLogged();
    private final SideShooterConstants sideShooterConstants;
    private final Logger logger = Logger.getInstance();

    private TrapezoidProfile angleMotorProfile = null;
    private double lastAngleMotorGenerationProfileTime;
    private Rotation2d generatedAngle = new Rotation2d();
    private SideShooterConstants.SideShooterState targetState = null;

    public static SideShooter getInstance() {
        return INSTANCE;
    }

    private SideShooter() {
        sideShooterIO = generateIO();
        sideShooterConstants = getConstants();
        setupCollectionCurrentWatcher();
    }

    @Override
    public void periodic() {
        sideShooterIO.updateInputs(sideShooterInputs);
        logger.processInputs("SideShooter", sideShooterInputs);
        updateMechanism();
    }

    /**
     * Sets whether the angle motor is in brake mode or not.
     *
     * @param brake whether the angle is in brake mode or not
     */
    public void setNeutralMode(boolean brake) {
        sideShooterIO.setNeutralMode(brake);
    }

    /**
     * Constructs a command that goes to the target state's angle, and then starts applying the state's power.
     *
     * @param sideShooterState the target side shooter state
     * @param byOrder          whether the command should reach the target state sequentially or parallelly
     * @return the command
     */
    public CommandBase getSetTargetShooterStateCommand(SideShooterConstants.SideShooterState sideShooterState, boolean byOrder) {
        final Rotation2d targetAngle = sideShooterState.angle;
        final double targetPower = sideShooterState.voltage;

        if (byOrder) {
            return new SequentialCommandGroup(
                    new InstantCommand(() -> targetState = sideShooterState),
                    getSetTargetShooterAngleCommand(targetAngle).until(() -> atAngle(targetAngle)),
                    Commands.withoutRequirements(getSetTargetShooterPowerCommand(targetPower))
                            .alongWith(getSetTargetShooterAngleCommand(targetAngle))
            );
        }

        final ParallelCommandGroup setTargetShooterStateCommand = new ParallelCommandGroup(
                new InstantCommand(() -> targetState = sideShooterState),
                Commands.withoutRequirements(getSetTargetShooterPowerCommand(targetPower)),
                getSetTargetShooterAngleCommand(targetAngle)
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
        return getCloseArmIfOpeningCommand(angle).andThen(new FunctionalCommand(
                () -> generateAngleMotorProfile(angle),
                this::setTargetAngleFromProfile,
                (interrupted) -> {
                },
                () -> false,
                this
        ));
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
                () -> {
                },
                this
        );
    }

    private CommandBase getCloseArmIfOpeningCommand(Rotation2d targetAngle) {
        return new ProxyCommand(() -> {
            if (canOpen(targetAngle))
                return new InstantCommand();

            final ArmConstants.ArmState defaultArmState = ArmConstants.ArmState.DEFAULT;
            return RobotContainer.ARM.getCurrentGoToArmPositionCommand(defaultArmState.elevatorPosition, defaultArmState.angle, 100, 100).until(() -> RobotContainer.ARM.atState(defaultArmState));
        });
    }

    private boolean canOpen(Rotation2d targetAngle) {
        logger.recordOutput("canOpen", targetAngle.equals(SideShooterConstants.SideShooterState.DEFAULT.angle) || RobotContainer.ARM.atState(ArmConstants.ArmState.DEFAULT));
        return targetAngle.equals(SideShooterConstants.SideShooterState.DEFAULT.angle) || RobotContainer.ARM.atState(ArmConstants.ArmState.DEFAULT);
    }

    private void generateAngleMotorProfile(Rotation2d targetAngle) {
        angleMotorProfile = new TrapezoidProfile(
                SideShooterConstants.ANGLE_CONSTRAINTS,
                new TrapezoidProfile.State(targetAngle.getDegrees(), 0),
                new TrapezoidProfile.State(sideShooterInputs.angleMotorPositionDegrees, 0)
        );

        lastAngleMotorGenerationProfileTime = Timer.getFPGATimestamp();
    }

    private void setupCollectionCurrentWatcher() {
        new CurrentWatcher(
                () -> sideShooterInputs.shootingMotorCurrent,
                SideShooterConstants.COLLECTION_CURRENT_THRESHOLD,
                SideShooterConstants.COLLECTION_TIME_THRESHOLD,
                () -> {
                    if (targetState == SideShooterConstants.SideShooterState.COLLECTION && getCurrentCommand() != null)
                        sideShooterIO.stopShootingMotor();
                }
        );
    }

    private void setTargetAngleFromProfile() {
        if (angleMotorProfile == null) {
            sideShooterIO.stopAngleMotor();
            return;
        }

        final TrapezoidProfile.State targetState = angleMotorProfile.calculate(getAngleMotorProfileTime());
        final double feedforward = sideShooterConstants.getAngleMotorFeedforward().calculate(Units.degreesToRadians(targetState.position), Units.degreesToRadians(targetState.velocity));
        final Rotation2d targetAngle = Rotation2d.fromDegrees(targetState.position);

        sideShooterIO.setTargetAngle(targetAngle, feedforward);
        generatedAngle = targetAngle;
    }

    private double getAngleMotorProfileTime() {
        return Timer.getFPGATimestamp() - lastAngleMotorGenerationProfileTime;
    }

    public boolean atAngle(Rotation2d angle) {
        return Math.abs(angle.getDegrees() - sideShooterInputs.angleMotorPositionDegrees) < SideShooterConstants.ANGLE_TOLERANCE_DEGREES;
    }

    private void updateMechanism() {
        SideShooterConstants.TARGET_SIDE_SHOOTER_LIGAMENT.setAngle(generatedAngle.getDegrees());
        SideShooterConstants.SIDE_SHOOTER_LIGAMENT.setAngle(sideShooterInputs.angleMotorPositionDegrees);

        logger.recordOutput("SideShooter/SideShooterMechanism", SideShooterConstants.SIDE_SHOOTER_MECHANISM);
        if (!Robot.IS_REAL) {
            logger.recordOutput("SideShooter/SideShooterPoses/sideShooterPose", getSideShooterPose());
            logger.recordOutput("SideShooter/SideShooterPoses/targetSideShooterPose", getTargetSideShooterPose());
        }
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

