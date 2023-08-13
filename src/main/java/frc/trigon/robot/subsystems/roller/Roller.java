package frc.trigon.robot.subsystems.roller;


import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.roller.simulationroller.SimulationRollerIO;
import frc.trigon.robot.subsystems.roller.staticroller.StaticRollerIO;

public class Roller extends SubsystemBase {
    private final static Roller INSTANCE = new Roller();
    private final RollerIO rollerIO;
    private final RollerInputsAutoLogged rollerInputs = new RollerInputsAutoLogged();

    public static Roller getInstance() {
        return INSTANCE;
    }

    private Roller() {
        rollerIO = generateIO();
    }

    // TODO: javadocs
    public ParallelCommandGroup getFullCloseCommand() {
        return getStopCollectingCommand().alongWith(getCloseCommand());
    }

    public ParallelCommandGroup getFullCollectionCommand() {
        return getStartCollectingCommand().alongWith(getOpenCommand());
    }

    public StartEndCommand getStartCollectingCommand() {
        return new StartEndCommand(
                this::startCollecting,
                () -> {},
                this
        );
    }

    public InstantCommand getStopCollectingCommand() {
        return new InstantCommand(rollerIO::stopCollectionMotor, this);
    }

    public FunctionalCommand getCloseCommand() {
        return new FunctionalCommand(
                this::close,
                this::close,
                (interrupted) -> rollerIO.stopAngleMotor(),
                this::isClosed,
                this
        );
    }

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
        rollerIO.setTargetAnglePower(RollerConstants.CLOSING_POWER);
    }

    private void open() {
        rollerIO.setTargetAnglePower(RollerConstants.OPENING_POWER);
    }

    private boolean isOpen() {
        return rollerInputs.angleMotorForwardLimitSwitchPressed;
    }

    private boolean isClosed() {
        return rollerInputs.angleMotorBackwardLimitSwitchPressed;
    }

    private RollerIO generateIO() {
        if (RobotConstants.IS_REPLAY)
            return new RollerIO();
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.STATIC)
            return new StaticRollerIO();

        return new SimulationRollerIO();
    }
}

