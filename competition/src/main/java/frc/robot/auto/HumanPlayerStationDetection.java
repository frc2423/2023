package frc.robot.auto;

import frc.robot.Robot;
import frc.robot.util.NtHelper;
import frc.robot.util.PhotonRunnable;
import frc.robot.util.PoseTransformUtils;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class HumanPlayerStationDetection {
    private static double maxDistance = 4; // in meters
    private static AprilTagFieldLayout layout = null;

    public static boolean seesTagCloseEnough() {
        if (layout == null) {
            try {
                layout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
                // PV estimates will always be blue, they'll get flipped by robot thread
                layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
            } catch(Exception e) {
    
            }
        } 

        var targetId = Robot.photonEstimator.grabBestID();

        if (targetId == null && layout != null) {
            NtHelper.setDouble("/testing/distanceBlue", 0);
            NtHelper.setDouble("/testing/distanceRed", 0);
            return false;
        }

        boolean isRed = Alliance.Red.equals(DriverStation.getAlliance());
        var maps = Waypoints.aprilTagsHumanPlayerStationLongName;


        var bluetag = layout.getTagPose(4).get().toPose2d();
        var redtag = PoseTransformUtils.transformRedPose(layout.getTagPose(5).get().toPose2d());
        var robotTranslation = Robot.m_drive.getPose().getTranslation();
        var distanceBlue = bluetag.getTranslation().getDistance(robotTranslation);
        var distanceRed = redtag.getTranslation().getDistance(robotTranslation);

        NtHelper.setDouble("/testing/distanceBlue", distanceBlue);
        NtHelper.setDouble("/testing/distanceRed", distanceRed);

        if (isRed && targetId == 5 && distanceRed <= maxDistance) {
            return true;
        }
        if (!isRed && targetId == 4 && distanceBlue <= maxDistance) {
            return true;
        }
        return false;
    }
}
