package frc.trigon.robot.subsystems.collector;

public class CollectorConstants {
    public enum CollectorState {
        STOP(0),
        COLLECT(1),
        FAST_COLLECT(1),
        EJECT(-1),
        FAST_EJECT(-1);

        CollectorState(double power) {
            this.power = power;
        }

        final double power;
    }
}
