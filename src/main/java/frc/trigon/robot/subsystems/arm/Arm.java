package frc.trigon.robot.subsystems.arm;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.Robot;
import frc.trigon.robot.commands.Commands;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.arm.kablamaarm.KablamaArmConstants;
import frc.trigon.robot.subsystems.arm.kablamaarm.KablamaArmIO;
import frc.trigon.robot.subsystems.arm.simulationarm.SimulationArmConstants;
import frc.trigon.robot.subsystems.arm.simulationarm.SimulationArmIO;
import frc.trigon.robot.subsystems.roller.Roller;
import frc.trigon.robot.subsystems.sideshooter.SideShooter;
import frc.trigon.robot.subsystems.sideshooter.SideShooterConstants;
import frc.trigon.robot.utilities.Conversions;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import java.util.HashMap;

public class Arm extends SubsystemBase {
    private final static Arm INSTANCE = new Arm();
    private final ArmIO armIO;
    private final ArmConstants armConstants;
    private final ArmInputsAutoLogged armInputs = new ArmInputsAutoLogged();
    private final Logger logger = Logger.getInstance();
    private final Roller roller = Roller.getInstance();
    private final SideShooter sideShooter = SideShooter.getInstance();

    private TrapezoidProfile
            angleMotorProfile = null,
            elevatorMotorProfile = null;
    private double lastAngleMotorGenerationProfileTime, lastElevatorMotorGenerationProfileTime;
    private Rotation2d
            targetAngle = new Rotation2d(),
            generatedAngle = new Rotation2d();
    private double targetElevatorPosition, generatedElevatorPosition;
    private ArmConstants.ArmState defaultState = null;
    private boolean isBraking = true;
    private double elevatorPositionAdder;
    private Rotation2d angleAdder = new Rotation2d();

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
        logger.processInputs("Arm", armInputs);
        updateDefaultCommand();
        updateMechanism();

        logger.recordOutput("Arm/angleAdder", angleAdder.getDegrees());
        logger.recordOutput("Arm/elevatorPositionAdder", elevatorPositionAdder);
    }
    
    public void setAngleAdder(Rotation2d angleAdder) {
        this.angleAdder = angleAdder;
    }
    
    public void setElevatorPositionAdder(double elevatorPositionAdder) {
        this.elevatorPositionAdder = elevatorPositionAdder;
    }

    public Rotation2d getAngleAdder() {
        return angleAdder;
    }

    public double getElevatorPositionAdder() {
        return elevatorPositionAdder;
    }

    public boolean isBraking() {
        return isBraking;
    }

    /**
     * Checks if the arm is at a state.
     *
     * @param state the state
     * @return whether the arm is at the state
     */
    public boolean atState(ArmConstants.ArmState state) {
        return elevatorMotorAtPosition(state.elevatorPosition + elevatorPositionAdder) && angleMotorAtAngle(state.angle.plus(angleAdder));
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
        setDefaultCommand(getCurrentGoToArmPositionCommand(defaultState.elevatorPosition, defaultState.angle, 100, 100));
    }

    /**
     * Sets whether the arm is in brake mode or not.
     *
     * @param brake whether the arm is in brake mode or not
     */
    public void setNeutralMode(boolean brake) {
        isBraking = brake;
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
        return new ProxyCommand(() -> {
            if (targetAngle.equals(ArmConstants.ArmState.DEFAULT.angle) || isSideClosed())
                getCurrentGoToArmPositionCommand(targetElevatorPosition, targetAngle, elevatorSpeedPercentage, angleSpeedPercentage);
            return Commands.withoutRequirements(getCloseSideShooterAndRollerCommand()).andThen(getCurrentGoToArmPositionCommand(targetElevatorPosition, targetAngle, elevatorSpeedPercentage, angleSpeedPercentage));
        });
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
        targetElevatorPosition = targetElevatorPosition + elevatorPositionAdder;
        targetAngle = targetAngle.plus(angleAdder);
        if (targetElevatorPosition < armInputs.elevatorMotorPositionRevolutions) {
            final double startAngle = armInputs.angleMotorPositionDegrees;
            return Commands.withRequirements(new SequentialCommandGroup(
                    getGoToElevatorPositionCommand(targetElevatorPosition, elevatorSpeedPercentage).alongWith(getGoToAngleCommand(Rotation2d.fromDegrees(startAngle), angleSpeedPercentage)).until(this::elevatorMotorCompletedMovement),
                    getGoToAngleCommand(targetAngle, angleSpeedPercentage).alongWith(getFollowElevatorProfileCommand())
            ), this);
        }

        return Commands.withRequirements(new SequentialCommandGroup(
                getGoToAngleCommand(targetAngle, angleSpeedPercentage).until(this::angleMotorCompletedMovement),
                getGoToElevatorPositionCommand(targetElevatorPosition, elevatorSpeedPercentage).alongWith(getGoToAngleCommand(targetAngle, angleSpeedPercentage))
        ), this);
    }

    private boolean isSideClosed() {
        return sideShooter.atAngle(SideShooterConstants.SideShooterState.DEFAULT.angle) && roller.isClosed();
    }

    private CommandBase getCloseSideShooterAndRollerCommand() {
        return new ParallelCommandGroup(
                sideShooter.getSetTargetShooterStateCommand(SideShooterConstants.SideShooterState.DEFAULT, false),
                roller.getFullCloseCommand()
        ).alongWith(
                getCurrentGoToArmPositionCommand(armInputs.elevatorMotorPositionRevolutions, Rotation2d.fromDegrees(armInputs.angleMotorPositionDegrees), 100, 100)
        ).until(
                () -> roller.isClosed() && sideShooter.atAngle(SideShooterConstants.SideShooterState.DEFAULT.angle)
        );
    }

    private void updateDefaultCommand() {
        if (defaultState == null || (getCurrentCommand() != null && getCurrentCommand().equals(getDefaultCommand())))
            return;
        setDefaultCommand(getCurrentGoToArmPositionCommand(defaultState.elevatorPosition, defaultState.angle, 100, 100));
    }

    private void updateMechanism() {
        ArmConstants.TARGET_ARM_LIGAMENT.setLength((generatedElevatorPosition * ArmConstants.THEORETICAL_METERS_PER_REVOLUTIONS) + ArmConstants.RETRACTED_ARM_LENGTH);
        ArmConstants.TARGET_ARM_LIGAMENT.setAngle(generatedAngle.getDegrees());
        ArmConstants.ARM_LIGAMENT.setLength((armInputs.elevatorMotorPositionRevolutions * ArmConstants.THEORETICAL_METERS_PER_REVOLUTIONS) + ArmConstants.RETRACTED_ARM_LENGTH);
        ArmConstants.ARM_LIGAMENT.setAngle(armInputs.angleMotorPositionDegrees);

        logger.recordOutput("Arm/ArmMechanism", ArmConstants.ARM_MECHANISM);
        if (!Robot.IS_REAL) {
            logger.recordOutput("Arm/ArmPoses/firstElevatorLevel", getFirstElevatorLevelPose());
            logger.recordOutput("Arm/ArmPoses/secondElevatorLevel", getSecondElevatorLevelPose());
            logger.recordOutput("Arm/ArmPoses/thirdElevatorLevel", getThirdElevatorLevelPose());

            logger.recordOutput("Arm/ArmPoses/targetFirstElevatorLevel", getTargetFirstElevatorLevelPose());
            logger.recordOutput("Arm/ArmPoses/targetSecondElevatorLevel", getTargetSecondElevatorLevelPose());
            logger.recordOutput("Arm/ArmPoses/targetThirdElevatorLevel", getTargetThirdElevatorLevelPose());
        }
    }

    private FunctionalCommand getGoToElevatorPositionCommand(double position, double speedPercentage) {
        return new FunctionalCommand(
                () -> generateElevatorMotorProfile(position, speedPercentage),
                this::setTargetElevatorPositionFromProfile,
                (interrupted) -> {},
                () -> false
        );
    }

    private FunctionalCommand getFollowElevatorProfileCommand() {
        return new FunctionalCommand(
                () -> {},
                this::setTargetElevatorPositionFromProfile,
                (interrupted) -> {},
                () -> false
        );
    }

    private FunctionalCommand getGoToAngleCommand(Rotation2d angle, double speedPercentage) {
        return new FunctionalCommand(
                () -> generateAngleMotorProfile(angle, speedPercentage),
                this::setTargetAngleFromProfile,
                (interrupted) -> {},
                () -> false
        );
    }

    private FunctionalCommand getFollowAngleProfileCommand() {
        return new FunctionalCommand(
                () -> {},
                this::setTargetAngleFromProfile,
                (interrupted) -> {},
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
        logger.recordOutput("Arm/generatedElevatorPosition", generatedElevatorPosition);
    }

    private void setTargetAngleFromProfile() {
        if (angleMotorProfile == null) {
            armIO.stopAngleMotor();
            return;
        }

        final TrapezoidProfile.State targetState = angleMotorProfile.calculate(getAngleMotorProfileTime());
        final double feedforward = getCurrentAngleFeedforward().calculate(Units.degreesToRadians(targetState.position), Units.degreesToRadians(targetState.velocity));
        final Rotation2d targetAngle = Rotation2d.fromDegrees(targetState.position);

        armIO.setTargetAngle(targetAngle, feedforward);
        generatedAngle = targetAngle;
    }

    private void generateElevatorMotorProfile(double targetElevatorPosition, double speedPercentage) {
        elevatorMotorProfile = new TrapezoidProfile(
                Conversions.scaleConstraints(ArmConstants.ELEVATOR_CONSTRAINTS, speedPercentage),
                new TrapezoidProfile.State(targetElevatorPosition, 0),
                new TrapezoidProfile.State(armInputs.elevatorMotorPositionRevolutions, armInputs.elevatorMotorVelocityRevolutionsPerSecond)
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

    private boolean elevatorMotorCompletedMovement() {
        return Math.abs(targetElevatorPosition - armInputs.elevatorMotorPositionRevolutions) < ArmConstants.ELEVATOR_MOTOR_COMPLETION_TOLERANCE;
    }

    private boolean angleMotorCompletedMovement() {
        return Math.abs(targetAngle.getDegrees() - armInputs.angleMotorPositionDegrees) < ArmConstants.ANGLE_MOTOR_COMPLETION_TOLERANCE;
    }

    private boolean elevatorMotorAtPosition(double position) {
        return Math.abs(position - armInputs.elevatorMotorPositionRevolutions) < ArmConstants.ELEVATOR_MOTOR_POSITION_TOLERANCE &&
                Math.abs(armInputs.elevatorMotorVelocityRevolutionsPerSecond) < ArmConstants.ELEVATOR_MOTOR_VELOCITY_TOLERANCE;
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
            if (Math.abs(currentHeight - armInputs.elevatorMotorPositionRevolutions) < Math.abs(closestHeight - armInputs.elevatorMotorPositionRevolutions))
                closestHeight = currentHeight;
        }

        logger.recordOutput("Arm/closestFFHeight", closestHeight);

        return heightToAngleMotorFeedforwardMap.get(closestHeight);
    }

    private Pose3d getThirdElevatorLevelPose() {
        final double elevatorPosition = armInputs.elevatorMotorPositionRevolutions * ArmConstants.THEORETICAL_METERS_PER_REVOLUTIONS;
        final Translation3d firstLevelToThirdLevelTranslation = new Translation3d(
                elevatorPosition + ArmConstants.THIRD_ELEVATOR_LEVEL_X_DIFFERENCE,
                0,
                ArmConstants.ELEVATOR_LEVELS_HEIGHT_DIFFERENCE
        );

        return getFirstElevatorLevelPose().transformBy(new Transform3d(firstLevelToThirdLevelTranslation, new Rotation3d()));
    }

    private Pose3d getTargetThirdElevatorLevelPose() {
        final double thirdLevelPosition = generatedElevatorPosition * ArmConstants.THEORETICAL_METERS_PER_REVOLUTIONS;
        final Translation3d firstLevelToThirdLevelTranslation = new Translation3d(
                thirdLevelPosition + ArmConstants.THIRD_ELEVATOR_LEVEL_X_DIFFERENCE,
                0,
                ArmConstants.ELEVATOR_LEVELS_HEIGHT_DIFFERENCE
        );

        return getFirstElevatorLevelPose().transformBy(new Transform3d(firstLevelToThirdLevelTranslation, new Rotation3d()));
    }

    private Pose3d getSecondElevatorLevelPose() {
        final double elevatorPosition = armInputs.elevatorMotorPositionRevolutions * ArmConstants.THEORETICAL_METERS_PER_REVOLUTIONS;
        final Translation3d firstLevelToSecondLevelTranslation = new Translation3d(
                MathUtil.clamp(elevatorPosition, 0, ArmConstants.SECOND_ELEVATOR_LEVEL_EXTENDED_LENGTH) + ArmConstants.SECOND_ELEVATOR_LEVEL_X_DIFFERENCE,
                0,
                ArmConstants.ELEVATOR_LEVELS_HEIGHT_DIFFERENCE
        );

        return getFirstElevatorLevelPose().transformBy(new Transform3d(firstLevelToSecondLevelTranslation, new Rotation3d()));
    }

    private Pose3d getTargetSecondElevatorLevelPose() {
        final Translation3d firstLevelToSecondLevelTranslation = new Translation3d(
                MathUtil.clamp(generatedElevatorPosition * ArmConstants.THEORETICAL_METERS_PER_REVOLUTIONS, 0, ArmConstants.SECOND_ELEVATOR_LEVEL_EXTENDED_LENGTH) + ArmConstants.SECOND_ELEVATOR_LEVEL_X_DIFFERENCE,
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

