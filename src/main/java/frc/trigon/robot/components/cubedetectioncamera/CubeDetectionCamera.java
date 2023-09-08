package frc.trigon.robot.components.cubedetectioncamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.Robot;
import org.littletonrobotics.junction.Logger;

/**
 * A class that represents a cube detection camera that detects cubes.
 */
public class CubeDetectionCamera extends SubsystemBase {
    private final CubeDetectionCameraInputsAutoLogged cubeDetectionCameraInputs = new CubeDetectionCameraInputsAutoLogged();
    private final CubeDetectionCameraIO cubeDetectionCameraIO;
    private final String hostname;

    /**
     * Constructs a new cube detection camera.
     *
     * @param hostname the name of the cube detection camera
     */
    public CubeDetectionCamera(String hostname) {
        this.hostname = hostname;
        cubeDetectionCameraIO = generateIO();
    }

    @Override
    public void periodic() {
        cubeDetectionCameraIO.updateInputs(cubeDetectionCameraInputs);
        Logger.getInstance().processInputs(hostname, cubeDetectionCameraInputs);
        Logger.getInstance().recordOutput(hostname + "/gamePiecePosition", getCubeYaw());
    }

    /**
     * @return whether the camera sees a cube
     */
    public boolean hasTargets() {
        return cubeDetectionCameraInputs.hasTargets;
    }

    /**
     * @return the yaw (x-axis position) of the target cube
     */
    public double getCubeYaw() {
        return cubeDetectionCameraInputs.bestCubeYaw;
    }

    private CubeDetectionCameraIO generateIO() {
        if (!Robot.IS_REAL)
            return new CubeDetectionCameraIO();

        return new PhotonCubeDetectionCameraIO(hostname);
    }
}
