package frc.robot.util;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.CameraConstants;

public class Camera {
    private PhotonCamera camera;

    private Transform3d cameraToRobot;

    public PhotonCamera returnCamera(){
        return camera;
    }

    public Camera(String name, Transform3d cameraToRobot) {
        camera = new PhotonCamera(name);
        this.cameraToRobot = cameraToRobot;
    }

    public double getTapeX() {
        // use the getYaw value
        // use a median filter to filter out bad values
        if(seesTarget()){
            return camera.getLatestResult().getBestTarget().getYaw();
        }
        else{
            return 0;
        }
    }

    public PhotonPipelineResult getLatestResult() {
        return camera.getLatestResult();
    }

    public boolean seesTarget() {
        return camera.getLatestResult().hasTargets();
        // assuming it's on the AprilTags pipeline
    }

    public double getDistanceFromID(int number) {
        if (returnTargetIDInfo(number) != null) {

            return PhotonUtils.calculateDistanceToTargetMeters(
                    cameraToRobot.getZ(),
                    CameraConstants.TARGET_HEIGHT_METERS,
                    cameraToRobot.getRotation().getY(),
                    Units.degreesToRadians(returnTargetIDInfo(number).getPitch()));

        }
        return -1;
    }

    public Transform3d getCameraToRobot() {
        return cameraToRobot;
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
