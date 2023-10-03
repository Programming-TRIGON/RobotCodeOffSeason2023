package frc.trigon.robot.subsystems.leds.ledcommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.trigon.robot.subsystems.leds.LedCommand;

public class MoveAndStopAtMiddleLedCommand extends LedCommand {
    private final Color movingColor, backgroundColor;;
    private final double cycleTime;
    private final int amountOfMovingLeds;
    
    public MoveAndStopAtMiddleLedCommand(Color movingColor, Color backgroundColor, double cycleTime, int amountOfMovingLeds) {
        this.movingColor = movingColor;
        this.backgroundColor = backgroundColor;
        this.cycleTime = cycleTime;
        this.amountOfMovingLeds = amountOfMovingLeds - 1;
    }

    @Override
    public void execute() {
        Color[] colors = new Color[ledsLength];
        int firstInMovingRange = getFirstInMovingRange(ledsLength);
        int lastInMovingRange = (firstInMovingRange + amountOfMovingLeds) % ledsLength;
        defineTheArrayOfTheColors(colors, firstInMovingRange, lastInMovingRange);
        setLedColors(colors);
    }

    private int getFirstInMovingRange(){
        return (int) (Timer.getFPGATimestamp() / cycleTime);
    }

    private int getFirstInMovingRange(int lengthOfStrip){
        return getFirstInMovingRange() % lengthOfStrip;
    }

    private boolean shouldBePrimeColor(int firstInMovingRange, int lastInMovingRange, int positionInLED) {
        return isPositionInLedInRange(firstInMovingRange, lastInMovingRange, positionInLED) ||
                isSplitByEnd(lastInMovingRange, positionInLED) ||
                isInvertedPositionInLedInRange(firstInMovingRange, lastInMovingRange, positionInLED) ||
                isInvertedSplitByEnd(lastInMovingRange, positionInLED);
    }

    private void defineTheArrayOfTheColors(Color[] colors, int firstInMovingRange, int lastInMovingRange) {
        for (int i = 0; i < ledsLength; i++) {
            if (shouldBePrimeColor(firstInMovingRange, lastInMovingRange, i)) {
                colors[(ledsLength - i) % ledsLength] = movingColor;
                colors[i] = movingColor;
            } else {
                colors[(ledsLength - i) % ledsLength] = backgroundColor;
                colors[i] = backgroundColor;
            }
        }
    }

    private boolean isPositionInLedInRange(int firstInMovingRange, int lastInMovingRange, int positionInLED) {
        return (positionInLED >= firstInMovingRange && positionInLED <= lastInMovingRange);
    }

    private boolean isSplitByEnd(int lastInMovingRange, int positionInLED) {
        return (positionInLED >= lastInMovingRange % ledsLength - amountOfMovingLeds &&
                positionInLED <= lastInMovingRange % ledsLength);
    }

    private boolean isInvertedPositionInLedInRange(int firstInMovingRange, int lastInMovingRange, int positionInLED) {
        return ((ledsLength - positionInLED) >= firstInMovingRange && (ledsLength - positionInLED) <= lastInMovingRange);
    }

    private boolean isInvertedSplitByEnd(int lastInMovingRange, int positionInLED) {
        return ((ledsLength - positionInLED) >= lastInMovingRange % ledsLength - amountOfMovingLeds &&
                (ledsLength - positionInLED) <= lastInMovingRange % ledsLength);
    }
}