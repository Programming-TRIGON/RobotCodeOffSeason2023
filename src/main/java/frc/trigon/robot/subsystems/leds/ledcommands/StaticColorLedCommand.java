package frc.trigon.robot.subsystems.leds.ledcommands;

import edu.wpi.first.wpilibj.util.Color;
import frc.trigon.robot.subsystems.leds.LedCommand;
import frc.trigon.robot.subsystems.leds.LedsConstants;

import java.util.Arrays;

public class StaticColorLedCommand extends LedCommand {
    private final Color staticColor;

    public StaticColorLedCommand(Color staticColor) {
        this.staticColor = staticColor;
    }

    @Override
    public void execute() {
        final Color[] colors = new Color[ledsLength];
        Arrays.fill(colors, staticColor);
        setLedColors(colors);
    }
}
