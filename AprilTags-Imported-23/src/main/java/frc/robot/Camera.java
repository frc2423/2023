package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class Camera {
    private PhotonCamera camera;
    public Camera(String name){
        camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
    }
    
    public PhotonPipelineResult getLatestResult(){
        return camera.getLatestResult();
    }

    public boolean haveTags(){
        return camera.getLatestResult().hasTargets();
        // assuming it's on the AprilTags pipeline
    }

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
}
