package frc.trigon.robot.utilities;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import frc.trigon.robot.constants.RobotConstants;

import java.io.Closeable;
import java.util.function.BooleanSupplier;

/**
 * A class that watches a limit switch and runs a callback when it is pressed for a certain amount of time.
 */
public class LimitSwitchWatcher implements Closeable {
    private final BooleanSupplier isPressedSupplier;
    private final double timeThreshold;
    private final boolean runOnce;
    private final Runnable callback;
    private final Notifier notifier;

    private double lastPressTimestamp;
    private boolean didRunCallback = false;

    /**
     * Constructs a new limit switch watcher.
     *
     * @param isPressedSupplier a supplier that returns whether the limit switch is pressed
     * @param timeThreshold     the amount of time the limit switch must be pressed for the callback to run
     * @param runOnce           whether the callback should only run once
     * @param callback          the callback to run
     */
    public LimitSwitchWatcher(BooleanSupplier isPressedSupplier, double timeThreshold, boolean runOnce, Runnable callback) {
        this.isPressedSupplier = isPressedSupplier;
        this.timeThreshold = timeThreshold;
        this.runOnce = runOnce;
        this.callback = callback;
        this.notifier = new Notifier(this::checkLimitSwitch);
        this.notifier.startPeriodic(RobotConstants.PERIODIC_TIME_SECONDS);
    }

    @Override
    public void close() {
        notifier.close();
    }

    private void checkLimitSwitch() {
        if (!isPressedSupplier.getAsBoolean()) {
            lastPressTimestamp = -1;
            didRunCallback = false;
            return;
        }
        if (isFirstPress())
            lastPressTimestamp = Timer.getFPGATimestamp();

        if (isTimeAboveThreshold())
            runCallback();
    }

    private void runCallback() {
        if (runOnce && didRunCallback)
            return;
        didRunCallback = true;

        if (callback != null)
            callback.run();
    }

    private boolean isTimeAboveThreshold() {
        return getTimeAboveThreshold() >= timeThreshold;
    }

    private double getTimeAboveThreshold() {
        return Timer.getFPGATimestamp() - lastPressTimestamp;
    }

    private boolean isFirstPress() {
        return lastPressTimestamp == -1;
    }
}
