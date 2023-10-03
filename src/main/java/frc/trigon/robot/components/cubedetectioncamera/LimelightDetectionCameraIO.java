package frc.trigon.robot.components.cubedetectioncamera;

import frc.trigon.robot.utilities.LimelightHelpers;

public class LimelightDetectionCameraIO extends CubeDetectionCameraIO {
    private final String hostname;

    public LimelightDetectionCameraIO(String hostname) {
        this.hostname = hostname;
    }

    @Override
    protected void updateInputs(CubeDetectionCameraInputsAutoLogged inputs) {
        inputs.hasTargets = LimelightHelpers.getTV(hostname);
        inputs.bestCubeYaw = getBestCubeYaw();
    }

    private double getBestCubeYaw() {
        final LimelightHelpers.LimelightResults latestResults = LimelightHelpers.getLatestResults(hostname);
        if (latestResults.targetingResults.targets_Detector.length == 0)
            return 0;
        return latestResults.targetingResults.targets_Detector[0].tx;
    }

}
