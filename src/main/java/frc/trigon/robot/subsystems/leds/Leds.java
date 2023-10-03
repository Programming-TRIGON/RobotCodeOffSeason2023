package frc.trigon.robot.subsystems.leds;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.Robot;

import java.util.Arrays;

public class Leds extends SubsystemBase {
    private final static Leds INSTANCE = new Leds();
    private final AddressableLED led = LedsConstants.LED;
    private final AddressableLEDBuffer ledBuffer = LedsConstants.LED_BUFFER;

    public static Leds getInstance() {
        return INSTANCE;
    }

    private Leds() {
    }

    void setLedColors(Color[] colors) {
        for (int i = 0; i < colors.length; i++) {
            final int ledIndex = LedsConstants.INVERTED ? colors.length - i - 1: i;

            if (!Robot.IS_REAL)
                ledBuffer.setLED(ledIndex, colors[i]);
            else
                ledBuffer.setLED(ledIndex, convertToRealColor(colors[i]));
        }
        led.setData(ledBuffer);
    }

    private Color convertToRealColor(Color color) {
        return rgbToGrb(balance(color));
    }

    private Color balance(Color color) {
        double v = Math.max(Math.max(color.red, color.green), color.blue);
        Color newColor = new Color(color.red, color.green / 2, color.blue / 4);
        double newV = Math.max(Math.max(newColor.red, newColor.green), newColor.blue);
        double ratio = v / newV;
        return new Color(newColor.red * ratio, newColor.green * ratio, newColor.blue * ratio);
    }

    private Color applyBrightness(Color color, double brightness) {
        if(brightness == 1)
            return color;
        if(brightness == 0)
            return Color.kBlack;
        return new Color(color.red * brightness, color.green * brightness, color.blue * brightness);
    }

    void turnOffLeds() {
        Color[] colors = new Color[LedsConstants.LEDS_LENGTH];
        Arrays.fill(colors, Color.kBlack);
        setLedColors(colors);
    }

    private Color rgbToGrb(Color color) {
        return new Color(color.green, color.red, color.blue);
    }
}

