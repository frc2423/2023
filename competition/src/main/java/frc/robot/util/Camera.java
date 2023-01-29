package frc.robot.util;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import frc.robot.constants.CameraConstants;

/**
 * Convenience class wrapping {@link org.photonvision.PhotonCamera} to simplify working with April Tags targets
 */
public class Camera {
    private PhotonCamera camera;
    /**
     * Constructs a new camera with a specified name
     * @param name The name to give the camera
     */
    public Camera(String name){
        camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
    }
    
    /**
     * @return The latest results from recongition procedure.  The return contains information about April Tags targets that have been identified in the scene.
     */
    public PhotonPipelineResult getLatestResult(){
        return camera.getLatestResult();
    }

    /**
     * @return True if the camera currently sees one or more targets.  False otherwise.
     */
    public boolean haveTags(){
        return camera.getLatestResult().hasTargets();
        // assuming it's on the AprilTags pipeline
    }


    /**
     * Takes in the id of an April Tags target that has been identified in the scene and returns the distance to the target.
     * @param number id of the target
     * @return the distance to the target in meters.  Or -1 if the the id is not a valid id from the scene
     */
    public double getDistanceFromID(int number){
        if (returnTargetIDInfo(number) != null) {
            
            return  PhotonUtils.calculateDistanceToTargetMeters(
                    CameraConstants.CAMERA_HEIGHT_METERS,
                    CameraConstants.TARGET_HEIGHT_METERS,
                    CameraConstants.CAMERA_PITCH_RADIANS,
                    Units.degreesToRadians(returnTargetIDInfo(number).getPitch()));

        }   
        return -1;
    }

    /**
     * @return true if the specified id is a valid April Tags target id in the scene.  False otherwise.
     */
    public boolean checkID(int id){
        if (getLatestResult().hasTargets()) {
            for (int i = 0; i < getLatestResult().targets.size(); i++) {
            if (getLatestResult().targets.get(i).getFiducialId() == id) {
                return true;
            } 
            
        } 
        }
        return false;
    }

    private PhotonTrackedTarget returnTargetIDInfo(int id){
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
