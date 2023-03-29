package frc.robot.util;

import java.io.IOException;
import java.util.concurrent.atomic.AtomicReference;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import frc.robot.constants.CameraConstants;

/**
 * Runnable that gets AprilTag data from PhotonVision.
 */
public class PhotonRunnable implements Runnable {

  private final PhotonPoseEstimator photonPoseEstimator;
  private final PhotonCamera photonCamera;
  private final AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose = new AtomicReference<EstimatedRobotPose>();

  private final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
  public static final double FIELD_LENGTH_METERS = 16.54175;
  public static final double FIELD_WIDTH_METERS = 8.0137;

  public PhotonRunnable() {
    this.photonCamera = Robot.m_camera.returnCamera();
    PhotonPoseEstimator photonPoseEstimator = null;
    
    try {
      var layout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
      // PV estimates will always be blue, they'll get flipped by robot thread
      layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
      if (photonCamera != null) {
        photonPoseEstimator = new PhotonPoseEstimator(
            layout, PoseStrategy.MULTI_TAG_PNP, photonCamera, CameraConstants.cameraToRobot.inverse());
      }
    } catch(IOException e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      photonPoseEstimator = null;
    }
    this.photonPoseEstimator = photonPoseEstimator;
  }

  @Override
  public void run() {      
    // Get AprilTag data
    if (photonPoseEstimator != null && photonCamera != null && !RobotState.isAutonomous()) {
      var photonResults = photonCamera.getLatestResult();
      if (photonResults.hasTargets() 
          && (photonResults.targets.size() > 1 || photonResults.targets.get(0).getPoseAmbiguity() < APRILTAG_AMBIGUITY_THRESHOLD)) { //how confident it is in the april tag
        photonPoseEstimator.update(photonResults).ifPresent(estimatedRobotPose -> {
          var estimatedPose = estimatedRobotPose.estimatedPose;
          // Make sure the measurement is on the field
          if (estimatedPose.getX() > 0.0 && estimatedPose.getX() <= FIELD_LENGTH_METERS
              && estimatedPose.getY() > 0.0 && estimatedPose.getY() <= FIELD_WIDTH_METERS) {
                if (Alliance.Red.equals(DriverStation.getAlliance())) {
                  double realX = FIELD_LENGTH_METERS - estimatedPose.getX();
                  double realY = FIELD_WIDTH_METERS - estimatedPose.getY();
                  Rotation3d realANGLE = estimatedPose.getRotation().plus(new Rotation3d(0,0, Math.PI));
                  Pose3d transformedPose = new Pose3d(realX, realY, estimatedPose.getZ(), realANGLE);
                  EstimatedRobotPose estimated = new EstimatedRobotPose(transformedPose, estimatedRobotPose.timestampSeconds, estimatedRobotPose.targetsUsed);
                  atomicEstimatedRobotPose.set(estimated);
                } else {
                  atomicEstimatedRobotPose.set(estimatedRobotPose);
                }
          }
        });
      }
    }  
  }

  /**
   * Gets the latest robot pose. Calling this will only return the pose once. If it returns a non-null value, it is a
   * new estimate that hasn't been returned before.
   * This pose will always be for the BLUE alliance. It must be flipped if the current alliance is RED.
   * @return latest estimated pose
   */
  public EstimatedRobotPose grabLatestEstimatedPose() {
    return atomicEstimatedRobotPose.getAndSet(null);
  }



}