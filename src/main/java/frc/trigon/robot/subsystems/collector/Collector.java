package frc.trigon.robot.subsystems.collector;


import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Collector extends SubsystemBase {
    private final static Collector INSTANCE = new Collector();
    private final CollectorIO collectorIO;

    public static Collector getInstance() {
        return INSTANCE;
    }

    private Collector() {
        collectorIO = generateIO();
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
                () -> {},
                this
        );
    }

    private void setTargetState(CollectorConstants.CollectorState state) {
        collectorIO.setTargetPower(state.power);
    }

    private CollectorIO generateIO() {
        // TODO: Make this actually generate the IO.
        return new CollectorIO();
    }
}

