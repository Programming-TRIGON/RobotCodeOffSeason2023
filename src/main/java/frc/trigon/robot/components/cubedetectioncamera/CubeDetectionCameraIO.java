package frc.trigon.robot.components.cubedetectioncamera;

import org.littletonrobotics.junction.AutoLog;

public class CubeDetectionCameraIO {
    protected CubeDetectionCameraIO() {
    }

    @AutoLog
    public static class CubeDetectionCameraInputs {
        public boolean hasTargets = false;
        public double bestCubeYaw = 0;
    }

    /**
     * Updates the inputs of the collection camera.
     *
     * @param inputs the inputs to update
     */
    protected void updateInputs(CubeDetectionCameraInputsAutoLogged inputs) {
    }
}
