package frc.robot.util;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import frc.robot.constants.CameraConstants;

public class Camera {
    private PhotonCamera camera;

    public Camera(String name) {
        camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
    }

    public double getTapeX() {
        // use the getYaw value
        // use a median filter to filter out bad values
        return 0;
    }

    public boolean seesTarget() {

        return false;
    }

    public PhotonPipelineResult getLatestResult() {
        return camera.getLatestResult();
    }

    public boolean haveTags() {
        return camera.getLatestResult().hasTargets();
        // assuming it's on the AprilTags pipeline
    }

    public double getDistanceFromID(int number) {
        if (returnTargetIDInfo(number) != null) {

            return PhotonUtils.calculateDistanceToTargetMeters(
                    CameraConstants.CAMERA_HEIGHT_METERS,
                    CameraConstants.TARGET_HEIGHT_METERS,
                    CameraConstants.CAMERA_PITCH_RADIANS,
                    Units.degreesToRadians(returnTargetIDInfo(number).getPitch()));

        }
        return -1;
    }

    public boolean checkID(int id) {
        if (getLatestResult().hasTargets()) {
            for (int i = 0; i < getLatestResult().targets.size(); i++) {
                if (getLatestResult().targets.get(i).getFiducialId() == id) {
                    return true;
                }

            }
        }
        return false;
    }

    private PhotonTrackedTarget returnTargetIDInfo(int id) {
        if (getLatestResult().hasTargets()) {
            for (int i = 0; i < getLatestResult().targets.size(); i++) {
                if (getLatestResult().targets.get(i).getFiducialId() == id) {
                    return getLatestResult().targets.get(i);
                }
            }
        }
        return null;
    }
}
