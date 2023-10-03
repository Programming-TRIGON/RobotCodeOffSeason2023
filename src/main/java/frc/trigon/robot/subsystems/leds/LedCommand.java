package frc.trigon.robot.subsystems.leds;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class LedCommand extends CommandBase {
    private final Leds leds = Leds.getInstance();
    protected int ledsLength = LedsConstants.LEDS_LENGTH;

    protected LedCommand() {
        addRequirements(leds);
    }

    @Override
    public void end(boolean interrupted) {
        leds.turnOffLeds();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    protected void setLedColors(Color[] colors) {
        leds.setLedColors(colors);
    }
}
