package frc.trigon.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LedsConstants {
    private static final int PORT = 9;
    static final boolean INVERTED = true;
    static final double BRIGHTNESS = 1;
    static final AddressableLED LED = new AddressableLED(PORT);
    static final int LEDS_LENGTH = 72;
    static final AddressableLEDBuffer LED_BUFFER = new AddressableLEDBuffer(LEDS_LENGTH);

    static {
        LED.setLength(LEDS_LENGTH);
        LED.setData(LED_BUFFER);
        LED.start();
    }
}
