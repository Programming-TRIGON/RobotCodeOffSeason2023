package frc.trigon.robot.subsystems.collector;


import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.collector.kablamacollector.KablamaCollectorIO;
import frc.trigon.robot.subsystems.collector.simulationcollector.SimulationCollectorIO;
import frc.trigon.robot.utilities.CurrentWatcher;
import org.littletonrobotics.junction.Logger;

public class Collector extends SubsystemBase {
    private final static Collector INSTANCE = new Collector();
    private final CollectorIO collectorIO;
    private final CollectorInputsAutoLogged collectorInputs = new CollectorInputsAutoLogged();
    private final Logger logger = Logger.getInstance();

    public static Collector getInstance() {
        return INSTANCE;
    }

    private Collector() {
        collectorIO = generateIO();
        new CurrentWatcher(
                () -> collectorInputs.current,
                10,
                0.2,
                collectorIO::stop
        );
    }

    @Override
    public void periodic() {
        collectorIO.updateInputs(collectorInputs);
        logger.processInputs("Collector", collectorInputs);
    }

    /**
     * Constructs a command that sets the target state of the collector.
     *
     * @param state the target state of the collector
     * @return the command
     */
    public StartEndCommand getSetTargetStateCommand(CollectorConstants.CollectorState state) {
        return new StartEndCommand(
                () -> setTargetState(state),
                collectorIO::stop,
                this
        );
    }

    private void setTargetState(CollectorConstants.CollectorState state) {
        collectorIO.setTargetPower(state.power);
    }

    private CollectorIO generateIO() {
        if (RobotConstants.IS_REPLAY)
            return new CollectorIO();
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.KABLAMA)
            return new KablamaCollectorIO();

        return new SimulationCollectorIO();
    }
}

