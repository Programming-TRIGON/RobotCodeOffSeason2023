package frc.trigon.robot.subsystems.arm;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.arm.simulationarm.SimulationArmConstants;
import frc.trigon.robot.subsystems.arm.simulationarm.SimulationArmIO;
import frc.trigon.robot.subsystems.arm.staticarm.StaticArmConstants;
import frc.trigon.robot.subsystems.arm.staticarm.StaticArmIO;
import frc.trigon.robot.utilities.Conversions;
import org.littletonrobotics.junction.Logger;

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
        Logger.getInstance().recordOutput("Arm/cmdName", this.getCurrentCommand() == null ? "null" : this.getCurrentCommand().getName());
    }

    /**
     * Sets whether the arm is in brake mode or not.
     *
     * @param brake whether the arm is in brake mode or not
     */
    public void setNeutralMode(boolean brake) {
        armIO.setNeutralMode(brake);
    }

    // TODO: javadocs
    public CommandBase getGoToArmStateCommand(ArmConstants.ArmStates targetState) {
        return getGoToArmStateCommand(targetState, 100, 100);
    }

    public CommandBase getGoToArmStateCommand(ArmConstants.ArmStates targetState, double elevatorSpeedPercentage, double angleSpeedPercentage) {
        return getGoToArmPositionCommand(targetState.elevatorPosition, targetState.angle, elevatorSpeedPercentage, angleSpeedPercentage);
    }

    public CommandBase getGoToArmPositionCommand(double targetElevatorPosition, Rotation2d targetAngle) {
        return getGoToArmPositionCommand(targetElevatorPosition, targetAngle, 100, 100);
    }

    public CommandBase getGoToArmPositionCommand(double targetElevatorPosition, Rotation2d targetAngle, double elevatorSpeedPercentage, double angleSpeedPercentage) {
        final ProxyCommand proxyCommand = new ProxyCommand(() -> getWantedArmLogicCommand(targetElevatorPosition, targetAngle, elevatorSpeedPercentage, angleSpeedPercentage));
        proxyCommand.addRequirements(this);
        return proxyCommand;
    }

    private CommandBase getWantedArmLogicCommand(double targetElevatorPosition, Rotation2d targetAngle, double elevatorSpeedPercentage, double angleSpeedPercentage) {
        if (angleMotorAtAngle(targetAngle)) {
            return new ParallelCommandGroup(
                    getGoToElevatorPositionCommand(targetElevatorPosition, elevatorSpeedPercentage),
                    getGoToAngleCommand(targetAngle, angleSpeedPercentage)
            );
        }

        if (isClosed()) {
            return new SequentialCommandGroup(
                    getGoToAngleCommand(targetAngle, angleSpeedPercentage).until(this::angleMotorAtGoal),
                    getGoToElevatorPositionCommand(targetElevatorPosition, elevatorSpeedPercentage).alongWith(getGoToAngleCommand(targetAngle, angleSpeedPercentage))
            );
        }

        if (targetElevatorPosition != 0) {
            return new SequentialCommandGroup(
                    getGoToElevatorPositionCommand(0, elevatorSpeedPercentage).until(this::elevatorMotorAtGoal).raceWith(getGoToAngleCommand(this.targetAngle, angleSpeedPercentage)),
                    getGoToAngleCommand(targetAngle, angleSpeedPercentage).until(this::angleMotorAtGoal),
                    getGoToElevatorPositionCommand(targetElevatorPosition, elevatorSpeedPercentage).alongWith(getGoToAngleCommand(targetAngle, angleSpeedPercentage))
            );
        }

        return new SequentialCommandGroup(
                getGoToElevatorPositionCommand(targetElevatorPosition, elevatorSpeedPercentage).until(this::elevatorMotorAtGoal).raceWith(getGoToAngleCommand(this.targetAngle, angleSpeedPercentage)),
                getGoToAngleCommand(targetAngle, angleSpeedPercentage).alongWith(getGoToElevatorPositionCommand(targetElevatorPosition, elevatorSpeedPercentage))
        );
    }

    private void updateMechanism() {
        ArmConstants.TARGET_ARM_LIGAMENT.setLength(generatedElevatorPosition + ArmConstants.RETRACTED_ARM_LENGTH);
        ArmConstants.TARGET_ARM_LIGAMENT.setAngle(generatedAngle.getDegrees());
        ArmConstants.ARM_LIGAMENT.setLength(armInputs.elevatorMotorPositionMeters + ArmConstants.RETRACTED_ARM_LENGTH);
        ArmConstants.ARM_LIGAMENT.setAngle(armInputs.angleMotorPositionDegrees);

        Logger.getInstance().recordOutput("Arm/ArmMechanism", ArmConstants.ARM_MECHANISM);
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
        final double feedforward = armConstants.getAngleMotorFeedforward().calculate(targetState.position, targetState.velocity);
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
        return Math.abs(targetElevatorPosition - armInputs.elevatorMotorPositionMeters) < ArmConstants.ELEVATOR_MOTOR_POSITION_TOLERANCE &&
                Math.abs(armInputs.elevatorMotorVelocityMetersPerSecond) < ArmConstants.ELEVATOR_MOTOR_VELOCITY_TOLERANCE;
    }

    private boolean angleMotorAtGoal() {
        return angleMotorAtAngle(targetAngle) && Math.abs(armInputs.angleMotorVelocityDegreesPerSecond) < ArmConstants.ANGLE_MOTOR_VELOCITY_TOLERANCE;
    }

    private boolean angleMotorAtAngle(Rotation2d angle) {
        return Math.abs(angle.getDegrees() - armInputs.angleMotorPositionDegrees) < ArmConstants.ANGLE_MOTOR_ANGLE_TOLERANCE;
    }

    private boolean isClosed() {
        return armInputs.elevatorMotorPositionMeters < ArmConstants.ELEVATOR_MOTOR_POSITION_TOLERANCE;
    }

    private ArmConstants generateConstants() {
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.STATIC)
            return new StaticArmConstants();

        return new SimulationArmConstants();
    }

    private ArmIO generateIO() {
        if (RobotConstants.IS_REPLAY)
            return new ArmIO();
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.STATIC)
            return new StaticArmIO();

        return new SimulationArmIO();
    }
}

