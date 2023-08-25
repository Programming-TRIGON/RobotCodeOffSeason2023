package frc.trigon.robot.components.cubedetectioncamera;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonCubeDetectionCameraIO extends CubeDetectionCameraIO {
    private final PhotonCamera photonCamera;

    /**
     * Constructs a new collection camera.
     *
     * @param hostname the name of the collection camera
     */
    protected PhotonCubeDetectionCameraIO(String hostname) {
        photonCamera = new PhotonCamera(hostname);
    }

    @Override
    protected void updateInputs(CubeDetectionCameraInputsAutoLogged inputs) {
        inputs.hasTargets = photonCamera.getLatestResult().hasTargets();
        inputs.bestCubeYaw = getTargetGamePiece().getYaw();
    }

    private PhotonTrackedTarget getTargetGamePiece() {
        return photonCamera.getLatestResult().getBestTarget();
    }
}
