package frc.trigon.robot.subsystems.leds.ledcommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.trigon.robot.subsystems.leds.LedCommand;

public class MovingColorLedCommand extends LedCommand {
    private final Color movingColor;
    private final Color backgroundColor;
    private final double cycleTime;
    private final int amountOfMovingLeds;

    public MovingColorLedCommand(Color movingColor, Color backgroundColor, double cycleTime, int amountOfMovingLeds) {
        this.movingColor = movingColor;
        this.backgroundColor = backgroundColor;
        this.cycleTime = cycleTime;
        this.amountOfMovingLeds = amountOfMovingLeds - 1;
    }

    @Override
    public void execute() {
        Color[] colors = new Color[ledsLength];
        int firstInMovingRange = getFirstInMovingRange(ledsLength);
        int lastInMovingRange = firstInMovingRange + amountOfMovingLeds;
        defineTheArrayOfTheColors(colors, firstInMovingRange, lastInMovingRange);
        setLedColors(colors);
    }

    private int getFirstInMovingRange(){
        return (int) (Timer.getFPGATimestamp() / cycleTime);
    }

    private int getFirstInMovingRange(int lengthOfStrip){
        return getFirstInMovingRange() % lengthOfStrip;
    }

    private void defineTheArrayOfTheColors(Color[] colors, int firstInMovingRange, int lastInMovingRange) {
        for (int i = 0; i < ledsLength; i++) {
            if (shouldBeMovingColor(firstInMovingRange, lastInMovingRange, i))
                colors[i] = movingColor;
            else
                colors[i] = backgroundColor;
        }
    }

    private boolean shouldBeMovingColor(int firstInMovingRange, int lastInMovingRange, int positionInLED) {
        return isPositionInLedInRange(firstInMovingRange, lastInMovingRange, positionInLED) ||
                isSplitByEnd(lastInMovingRange, positionInLED);
    }

    private boolean isPositionInLedInRange(int firstInMovingRange, int lastInMovingRange, int positionInLED) {
        return positionInLED >= firstInMovingRange && positionInLED <= lastInMovingRange;
    }

    private boolean isSplitByEnd(int lastInMovingRange, int positionInLED) {
        return positionInLED >= lastInMovingRange % ledsLength - amountOfMovingLeds &&
                positionInLED <= lastInMovingRange % ledsLength;
    }
}
