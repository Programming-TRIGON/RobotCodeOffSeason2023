package frc.trigon.robot.subsystems.arm;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.commands.Commands;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.arm.kablamaarm.KablamaArmConstants;
import frc.trigon.robot.subsystems.arm.kablamaarm.KablamaArmIO;
import frc.trigon.robot.subsystems.arm.simulationarm.SimulationArmConstants;
import frc.trigon.robot.subsystems.arm.simulationarm.SimulationArmIO;
import frc.trigon.robot.utilities.Conversions;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;

public class Arm extends SubsystemBase {
    private final static Arm INSTANCE = new Arm();
    private final ArmIO armIO;
    private final ArmConstants armConstants;
    private final ArmInputsAutoLogged armInputs = new ArmInputsAutoLogged();

    private TrapezoidProfile
            angleMotorProfile = null,
            elevatorMotorProfile = null;
    private double lastAngleMotorGenerationProfileTime, lastElevatorMotorGenerationProfileTime;
    private Rotation2d
            targetAngle = new Rotation2d(),
            generatedAngle = new Rotation2d();
    private double targetElevatorPosition, generatedElevatorPosition;
    private ArmConstants.ArmState defaultState = null;

    public static Arm getInstance() {
        return INSTANCE;
    }

    private Arm() {
        armIO = generateIO();
        armConstants = generateConstants();
    }

    @Override
    public void periodic() {
        armIO.updateInputs(armInputs);
        Logger.getInstance().processInputs("Arm", armInputs);
        updateMechanism();
        updateDefaultCommand();
    }

    /**
     * Checks if the arm is at a state.
     *
     * @param state the state
     * @return whether the arm is at the state
     */
    public boolean atState(ArmConstants.ArmState state) {
        return elevatorMotorAtPosition(state.elevatorPosition) && angleMotorAtAngle(state.angle);
    }

    /**
     * @return if the arm is at its goal
     */
    public boolean atGoal() {
        return angleMotorAtGoal() && elevatorMotorAtGoal();
    }

    /**
     * Sets the default state of the arm.
     *
     * @param state the default state
     */
    public void setDefaultState(ArmConstants.ArmState state) {
        defaultState = state;
    }

    /**
     * Sets whether the arm is in brake mode or not.
     *
     * @param brake whether the arm is in brake mode or not
     */
    public void setNeutralMode(boolean brake) {
        armIO.setNeutralMode(brake);
    }

    /**
     * Constructs a command that goes to an arm state.
     *
     * @param targetState the target arm state
     * @return the command
     */
    public CommandBase getGoToArmStateCommand(ArmConstants.ArmState targetState) {
        return getGoToArmStateCommand(targetState, 100, 100);
    }

    /**
     * Constructs a command that goes to an arm state.
     *
     * @param targetState             the target arm state
     * @param elevatorSpeedPercentage the elevator speed percentage
     * @param angleSpeedPercentage    the angle motor speed percentage
     * @return the command
     */
    public CommandBase getGoToArmStateCommand(ArmConstants.ArmState targetState, double elevatorSpeedPercentage, double angleSpeedPercentage) {
        return getGoToArmPositionCommand(targetState.elevatorPosition, targetState.angle, elevatorSpeedPercentage, angleSpeedPercentage);
    }

    /**
     * Constructs a command that goes to an arm position.
     *
     * @param targetElevatorPosition the target elevator position
     * @param targetAngle            the target angle of the arm
     * @return the command
     */
    public CommandBase getGoToArmPositionCommand(double targetElevatorPosition, Rotation2d targetAngle) {
        return getGoToArmPositionCommand(targetElevatorPosition, targetAngle, 100, 100);
    }

    /**
     * Constructs a command that goes to an arm position.
     *
     * @param targetElevatorPosition  the target elevator position
     * @param targetAngle             the target angle of the arm
     * @param elevatorSpeedPercentage the elevator speed percentage
     * @param angleSpeedPercentage    the angle motor speed percentage
     * @return the command
     */
    public CommandBase getGoToArmPositionCommand(double targetElevatorPosition, Rotation2d targetAngle, double elevatorSpeedPercentage, double angleSpeedPercentage) {
        return new ProxyCommand(() -> getCurrentGoToArmPositionCommand(targetElevatorPosition, targetAngle, elevatorSpeedPercentage, angleSpeedPercentage));
    }

    /**
     * Constructs a command that goes the target am position, using the logic to get there relative to the current arm position.
     * WARNING! If this is not ran almost instantly the logic might be wrong!
     *
     * @param targetElevatorPosition  the target elevator position
     * @param targetAngle             the target angle of the arm
     * @param elevatorSpeedPercentage the elevator speed percentage
     * @param angleSpeedPercentage    the angle motor speed percentage
     * @return the command
     */
    public CommandBase getCurrentGoToArmPositionCommand(double targetElevatorPosition, Rotation2d targetAngle, double elevatorSpeedPercentage, double angleSpeedPercentage) {
        if (targetElevatorPosition < armInputs.elevatorMotorPositionMeters) {
            return Commands.withRequirements(new SequentialCommandGroup(
                    getGoToElevatorPositionCommand(targetElevatorPosition, elevatorSpeedPercentage).alongWith(getGoToAngleCommand(Rotation2d.fromDegrees(armInputs.angleMotorPositionDegrees), angleSpeedPercentage)).until(this::elevatorMotorAtGoal),
                    getGoToAngleCommand(targetAngle, angleSpeedPercentage).alongWith(getGoToElevatorPositionCommand(targetElevatorPosition, elevatorSpeedPercentage))
            ), this);
        }

        return Commands.withRequirements(new SequentialCommandGroup(
                getGoToAngleCommand(targetAngle, angleSpeedPercentage).until(this::angleMotorAtGoal),
                getGoToElevatorPositionCommand(targetElevatorPosition, elevatorSpeedPercentage).alongWith(getGoToAngleCommand(targetAngle, angleSpeedPercentage))
        ), this);
    }

    private void updateDefaultCommand() {
        if (defaultState == null || (getCurrentCommand() != null && getCurrentCommand().equals(getDefaultCommand())))
            return;
        setDefaultCommand(getCurrentGoToArmPositionCommand(defaultState.elevatorPosition, defaultState.angle, 100, 100));
    }

    private void updateMechanism() {
        ArmConstants.TARGET_ARM_LIGAMENT.setLength(generatedElevatorPosition + ArmConstants.RETRACTED_ARM_LENGTH);
        ArmConstants.TARGET_ARM_LIGAMENT.setAngle(generatedAngle.getDegrees());
        ArmConstants.ARM_LIGAMENT.setLength(armInputs.elevatorMotorPositionMeters + ArmConstants.RETRACTED_ARM_LENGTH);
        ArmConstants.ARM_LIGAMENT.setAngle(armInputs.angleMotorPositionDegrees);

        Logger.getInstance().recordOutput("Arm/ArmMechanism", ArmConstants.ARM_MECHANISM);
        Logger.getInstance().recordOutput("Arm/ArmPoses/firstElevatorLevel", getFirstElevatorLevelPose());
        Logger.getInstance().recordOutput("Arm/ArmPoses/secondElevatorLevel", getSecondElevatorLevelPose());
        Logger.getInstance().recordOutput("Arm/ArmPoses/thirdElevatorLevel", getThirdElevatorLevelPose());

        Logger.getInstance().recordOutput("Arm/ArmPoses/targetFirstElevatorLevel", getTargetFirstElevatorLevelPose());
        Logger.getInstance().recordOutput("Arm/ArmPoses/targetSecondElevatorLevel", getTargetSecondElevatorLevelPose());
        Logger.getInstance().recordOutput("Arm/ArmPoses/targetThirdElevatorLevel", getTargetThirdElevatorLevelPose());
    }

    private FunctionalCommand getGoToElevatorPositionCommand(double position, double speedPercentage) {
        return new FunctionalCommand(
                () -> generateElevatorMotorProfile(position, speedPercentage),
                this::setTargetElevatorPositionFromProfile,
                (interrupted) -> armIO.stopElevatorMotor(),
                () -> false
        );
    }

    private FunctionalCommand getGoToAngleCommand(Rotation2d angle, double speedPercentage) {
        return new FunctionalCommand(
                () -> generateAngleMotorProfile(angle, speedPercentage),
                this::setTargetAngleFromProfile,
                (interrupted) -> armIO.stopAngleMotor(),
                () -> false
        );
    }

    private void setTargetElevatorPositionFromProfile() {
        if (elevatorMotorProfile == null) {
            armIO.stopElevatorMotor();
            return;
        }

        final TrapezoidProfile.State targetState = elevatorMotorProfile.calculate(getElevatorMotorProfileTime());
        final double feedforward = armConstants.getElevatorMotorFeedforward().calculate(targetState.velocity);

        armIO.setTargetElevatorPosition(targetState.position, feedforward);
        generatedElevatorPosition = targetState.position;
    }

    private void setTargetAngleFromProfile() {
        if (angleMotorProfile == null) {
            armIO.stopAngleMotor();
            return;
        }

        final TrapezoidProfile.State targetState = angleMotorProfile.calculate(getAngleMotorProfileTime());
        final double feedforward = getCurrentAngleFeedforward().calculate(targetState.position, targetState.velocity);
        final Rotation2d targetAngle = Rotation2d.fromDegrees(targetState.position);

        armIO.setTargetAngle(targetAngle, feedforward);
        generatedAngle = targetAngle;
    }

    private void generateElevatorMotorProfile(double targetElevatorPosition, double speedPercentage) {
        elevatorMotorProfile = new TrapezoidProfile(
                Conversions.scaleConstraints(ArmConstants.ELEVATOR_CONSTRAINTS, speedPercentage),
                new TrapezoidProfile.State(targetElevatorPosition, 0),
                new TrapezoidProfile.State(armInputs.elevatorMotorPositionMeters, armInputs.elevatorMotorVelocityMetersPerSecond)
        );
        this.targetElevatorPosition = targetElevatorPosition;

        lastElevatorMotorGenerationProfileTime = Timer.getFPGATimestamp();
    }

    private void generateAngleMotorProfile(Rotation2d targetAngle, double speedPercentage) {
        angleMotorProfile = new TrapezoidProfile(
                Conversions.scaleConstraints(ArmConstants.ANGLE_CONSTRAINTS, speedPercentage),
                new TrapezoidProfile.State(targetAngle.getDegrees(), 0),
                new TrapezoidProfile.State(armInputs.angleMotorPositionDegrees, armInputs.angleMotorVelocityDegreesPerSecond)
        );
        this.targetAngle = targetAngle;

        lastAngleMotorGenerationProfileTime = Timer.getFPGATimestamp();
    }

    private double getElevatorMotorProfileTime() {
        return Timer.getFPGATimestamp() - lastElevatorMotorGenerationProfileTime;
    }

    private double getAngleMotorProfileTime() {
        return Timer.getFPGATimestamp() - lastAngleMotorGenerationProfileTime;
    }

    private boolean elevatorMotorAtGoal() {
        return elevatorMotorAtPosition(targetElevatorPosition);
    }

    private boolean angleMotorAtGoal() {
        return angleMotorAtAngle(targetAngle);
    }

    private boolean elevatorMotorAtPosition(double position) {
        return Math.abs(position - armInputs.elevatorMotorPositionMeters) < ArmConstants.ELEVATOR_MOTOR_POSITION_TOLERANCE &&
                Math.abs(armInputs.elevatorMotorVelocityMetersPerSecond) < ArmConstants.ELEVATOR_MOTOR_VELOCITY_TOLERANCE;
    }

    private boolean angleMotorAtAngle(Rotation2d angle) {
        return Math.abs(angle.getDegrees() - armInputs.angleMotorPositionDegrees) < ArmConstants.ANGLE_MOTOR_ANGLE_TOLERANCE &&
                Math.abs(armInputs.angleMotorVelocityDegreesPerSecond) < ArmConstants.ANGLE_MOTOR_VELOCITY_TOLERANCE;
    }

    private ArmFeedforward getCurrentAngleFeedforward() {
        final HashMap<Double, ArmFeedforward> heightToAngleMotorFeedforwardMap = armConstants.getHeightToAngleMotorFeedforwardMap();
        double closestHeight = 0;
        final int mapSize = heightToAngleMotorFeedforwardMap.size();

        for (int i = 0; i < mapSize; i++) {
            final double currentHeight = heightToAngleMotorFeedforwardMap.keySet().toArray(new Double[mapSize])[i];
            if (i == 0) {
                closestHeight = currentHeight;
                continue;
            }
            if (Math.abs(currentHeight - armInputs.elevatorMotorPositionMeters) < Math.abs(closestHeight - armInputs.elevatorMotorPositionMeters)) {
                closestHeight = currentHeight;
            }
        }

        return heightToAngleMotorFeedforwardMap.get(closestHeight);
    }

    private Pose3d getThirdElevatorLevelPose() {
        final double elevatorPosition = armInputs.elevatorMotorPositionMeters;
        final Translation3d firstLevelToThirdLevelTranslation = new Translation3d(
                elevatorPosition + ArmConstants.THIRD_ELEVATOR_LEVEL_X_DIFFERENCE,
                0,
                ArmConstants.ELEVATOR_LEVELS_HEIGHT_DIFFERENCE
        );

        return getFirstElevatorLevelPose().transformBy(new Transform3d(firstLevelToThirdLevelTranslation, new Rotation3d()));
    }

    private Pose3d getTargetThirdElevatorLevelPose() {
        final double thirdLevelPosition = generatedElevatorPosition;
        final Translation3d firstLevelToThirdLevelTranslation = new Translation3d(
                thirdLevelPosition + ArmConstants.THIRD_ELEVATOR_LEVEL_X_DIFFERENCE,
                0,
                ArmConstants.ELEVATOR_LEVELS_HEIGHT_DIFFERENCE
        );

        return getFirstElevatorLevelPose().transformBy(new Transform3d(firstLevelToThirdLevelTranslation, new Rotation3d()));
    }

    private Pose3d getSecondElevatorLevelPose() {
        final double elevatorPosition = armInputs.elevatorMotorPositionMeters;
        final Translation3d firstLevelToSecondLevelTranslation = new Translation3d(
                MathUtil.clamp(elevatorPosition, 0, ArmConstants.SECOND_ELEVATOR_LEVEL_EXTENDED_LENGTH) + ArmConstants.SECOND_ELEVATOR_LEVEL_X_DIFFERENCE,
                0,
                ArmConstants.ELEVATOR_LEVELS_HEIGHT_DIFFERENCE
        );

        return getFirstElevatorLevelPose().transformBy(new Transform3d(firstLevelToSecondLevelTranslation, new Rotation3d()));
    }

    private Pose3d getTargetSecondElevatorLevelPose() {
        final Translation3d firstLevelToSecondLevelTranslation = new Translation3d(
                MathUtil.clamp(generatedElevatorPosition, 0, ArmConstants.SECOND_ELEVATOR_LEVEL_EXTENDED_LENGTH) + ArmConstants.SECOND_ELEVATOR_LEVEL_X_DIFFERENCE,
                0,
                ArmConstants.ELEVATOR_LEVELS_HEIGHT_DIFFERENCE
        );

        return getFirstElevatorLevelPose().transformBy(new Transform3d(firstLevelToSecondLevelTranslation, new Rotation3d()));
    }

    private Pose3d getFirstElevatorLevelPose() {
        return new Pose3d(
                ArmConstants.ARM_ROOT_TRANSLATION,
                new Rotation3d(0, -Units.degreesToRadians(armInputs.angleMotorPositionDegrees), 0)
        );
    }

    private Pose3d getTargetFirstElevatorLevelPose() {
        return new Pose3d(
                ArmConstants.ARM_ROOT_TRANSLATION,
                new Rotation3d(0, -generatedAngle.getRadians(), 0)
        );
    }

    private ArmConstants generateConstants() {
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.KABLAMA)
            return new KablamaArmConstants();

        return new SimulationArmConstants();
    }

    private ArmIO generateIO() {
        if (RobotConstants.IS_REPLAY)
            return new ArmIO();
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.KABLAMA)
            return new KablamaArmIO();

        return new SimulationArmIO();
    }
}

