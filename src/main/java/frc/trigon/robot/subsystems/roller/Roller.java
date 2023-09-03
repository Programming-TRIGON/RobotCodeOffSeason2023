package frc.trigon.robot.subsystems.roller;


import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.commands.Commands;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.roller.simulationroller.SimulationRollerIO;
import frc.trigon.robot.subsystems.roller.kablamaroller.KablamaRollerIO;
import org.littletonrobotics.junction.Logger;

public class Roller extends SubsystemBase {
    private final static Roller INSTANCE = new Roller();
    private final RollerIO rollerIO;
    private final RollerInputsAutoLogged rollerInputs = new RollerInputsAutoLogged();
    private boolean isTargetStateClosed = false;

    public static Roller getInstance() {
        return INSTANCE;
    }

    private Roller() {
        rollerIO = generateIO();
    }

    @Override
    public void periodic() {
        rollerIO.updateInputs(rollerInputs);
        Logger.getInstance().processInputs("Roller", rollerInputs);
        updateMechanism();
    }

    /**
     * @return a command that fully closes the roller, which means closing the roller and stopping collection
     */
    public CommandBase getFullCloseCommand() {
        final ParallelCommandGroup fullCloseCommand = new ParallelCommandGroup(
                Commands.withoutRequirements(getCloseCommand()),
                Commands.withoutRequirements(getStopCollectingCommand())
        );

        return Commands.withRequirements(fullCloseCommand, this);
    }

    /**
     * @return a command that fully activates the collection system, which means opening the roller and applying the collection power
     */
    public CommandBase getFullCollectionCommand() {
        final SequentialCommandGroup fullCollectionCommand = new SequentialCommandGroup(
                getOpenCommand().until(this::isOpen),
                getStartCollectingCommand()
        );

        return Commands.withRequirements(fullCollectionCommand, this);
    }

    /**
     * @return a command that applies the collection power
     */
    public StartEndCommand getStartCollectingCommand() {
        return new StartEndCommand(
                this::startCollecting,
                () -> {},
                this
        );
    }

    /**
     * @return a command that stops the collection motor
     */
    public InstantCommand getStopCollectingCommand() {
        return new InstantCommand(rollerIO::stopCollectionMotor, this);
    }

    /**
     * @return a command that closes the roller
     */
    public FunctionalCommand getCloseCommand() {
        return new FunctionalCommand(
                () -> {},
                this::close,
                (interrupted) -> rollerIO.stopAngleMotor(),
                this::isClosed,
                this
        );
    }

    /**
     * @return a command that opens the roller
     */
    public FunctionalCommand getOpenCommand() {
        return new FunctionalCommand(
                this::open,
                this::open,
                (interrupted) -> rollerIO.stopAngleMotor(),
                this::isOpen
        );
    }

    private void startCollecting() {
        rollerIO.setTargetCollectionPower(RollerConstants.COLLECTING_POWER);
    }

    private void close() {
        isTargetStateClosed = true;
        rollerIO.setTargetAnglePower(RollerConstants.CLOSING_POWER);
    }

    private void open() {
        isTargetStateClosed = false;
        rollerIO.setTargetAnglePower(RollerConstants.OPENING_POWER);
    }

    public boolean isOpen() {
        return rollerInputs.angleMotorForwardLimitSwitchPressed;
    }

    public boolean isClosed() {
        return rollerInputs.angleMotorBackwardLimitSwitchPressed;
    }

    private void updateMechanism() {
        RollerConstants.TARGET_ROLLER_LIGAMENT.setAngle(isTargetStateClosed ? 90 : 0);
        RollerConstants.ROLLER_LIGAMENT.setAngle(isClosed() ? 90 : 0);

        Logger.getInstance().recordOutput("Roller/RollerMechanism", RollerConstants.ROLLER_MECHANISM);
        Logger.getInstance().recordOutput("Roller/RollerPoses/rollerPose", getRollerPose());
        Logger.getInstance().recordOutput("Roller/RollerPoses/targetRollerPose", getTargetRollerPose());
    }

    private Pose3d getRollerPose() {
        return new Pose3d(
                RollerConstants.ROLLER_TRANSLATION,
                new Rotation3d(0, isClosed() ? Units.degreesToRadians(-90) : 0, RollerConstants.ROLLER_YAW)
        );
    }

    private Pose3d getTargetRollerPose() {
        return new Pose3d(
                RollerConstants.ROLLER_TRANSLATION,
                new Rotation3d(0, isTargetStateClosed ? Units.degreesToRadians(-90) : 0, RollerConstants.ROLLER_YAW)
        );
    }

    private RollerIO generateIO() {
        if (RobotConstants.IS_REPLAY)
            return new RollerIO();
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.KABLAMA)
            return new KablamaRollerIO();

        return new SimulationRollerIO();
    }
}

