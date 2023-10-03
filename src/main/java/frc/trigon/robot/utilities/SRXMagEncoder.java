package frc.trigon.robot.utilities;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.MathUtil;

// TODO: javadocs
public class SRXMagEncoder {
    private final WPI_TalonSRX magEncoder;
    private double offset = 0;
    private double
            positionScale = Conversions.MAG_TICKS,
            velocityScale = Conversions.MAG_TICKS;
    private double minPos, maxPos;
    private boolean continuousPositionEnabled = false;

    public SRXMagEncoder(int id) {
        this(new WPI_TalonSRX(id));
    }

    public SRXMagEncoder(WPI_TalonSRX magEncoder) {
        this.magEncoder = magEncoder;
        magEncoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    }

    public void enableContinuousPosition(double minPos, double maxPos) {
        continuousPositionEnabled = true;
        this.minPos = minPos;
        this.maxPos = maxPos;
    }

    public void setOffset(double offset) {
        this.offset = offset;
    }

    public void setPositionScale(double scale) {
        positionScale = scale;
    }

    public void setVelocityScale(double scale) {
        velocityScale = scale;
    }

    public ErrorCode configFactoryDefault() {
        return magEncoder.configFactoryDefault();
    }

    public void setSensorPhase(boolean PhaseSensor) {
        magEncoder.setSensorPhase(PhaseSensor);
    }

    public double getPosition() {
        final double positionRevolutions = Conversions.magTicksToRevolutions(magEncoder.getSelectedSensorPosition());
        final double scaledPosition = positionRevolutions * positionScale;
        final double offsettedPosition = Conversions.offsetRead(scaledPosition, offset);

        if (continuousPositionEnabled)
            return MathUtil.inputModulus(offsettedPosition, minPos, maxPos);
        return offsettedPosition;
    }

    public ErrorCode setSelectedSensorPosition(double sensorPos, int timeoutMs) {
        final double positionMagTicks = Conversions.revolutionsToMagTicks(sensorPos / positionScale);
        final double offsettedPosition = Conversions.offsetWrite(positionMagTicks, Conversions.revolutionsToMagTicks(offset / positionMagTicks));
        return magEncoder.setSelectedSensorPosition(offsettedPosition, 0, timeoutMs);
    }

    public ErrorCode setSelectedSensorPosition(double sensorPos) {
        return setSelectedSensorPosition(sensorPos, 0);
    }

    public double getVelocity() {
        final double velocityMagTicksPerSecond = Conversions.perHundredMsToPerSecond(magEncoder.getSelectedSensorVelocity());
        final double velocityRevolutionsPerSecond = Conversions.magTicksToRevolutions(velocityMagTicksPerSecond);
        return velocityRevolutionsPerSecond * velocityScale;
    }
}
