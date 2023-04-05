package frc.robot.auto;
import frc.robot.Robot;
import frc.robot.util.PhotonRunnable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
public class HumanPlayerStationDetection {
    private static double mindistance = 10;
    public static boolean seesTagCloseEnough() {
   
        var targetId = Robot.photonEstimator.grabBestID();
        if (targetId == null) {
            return false;
        }

        boolean isRed = Alliance.Red.equals(DriverStation.getAlliance());
        var maps = Waypoints.aprilTagsHumanPlayerStationLongName;
        
        var bluetag = maps.get(5);
        var redtag = maps.get(4);
        var pose = Robot.m_drive.getPose();
        var distanceblue = bluetag.getTranslation().getDistance(pose.getTranslation());
        var distancered = redtag.getTranslation().getDistance(pose.getTranslation());

        if (isRed && targetId == 5 && distancered <=mindistance) {
            return true;
        }
        if (!isRed && targetId == 4 && distanceblue <=mindistance ) {
            return true;
        }
        return false;
    }
}
