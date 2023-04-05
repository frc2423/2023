package frc.robot.auto;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
public class HumanPlayerStationDetection {
    private static double mindistance = 10;
    public static boolean seeapriltag() {
        var m_Camera = Robot.m_camera;
        var res = m_Camera.returnCamera().getLatestResult();
        // var redhp = Waypoints.RED_HP;

        boolean isRed = Alliance.Red.equals(DriverStation.getAlliance());
        if (!res.hasTargets()) {
            return false;
        }
        var maps = Waypoints.aprilTagsHumanPlayerStationLongName;
        
        var bluetag = maps.get(5);
        var redtag = maps.get(4);
        var pose = Robot.m_drive.getPose();
        var distanceblue = bluetag.getTranslation().getDistance(pose.getTranslation());
        var distancered = redtag.getTranslation().getDistance(pose.getTranslation());
         var bestTarget = res.getBestTarget();
        var targetid = bestTarget.getFiducialId();
        
        System.out.println(distancered);
        
        System.out.println(distanceblue);
        if (isRed && targetid == 4 && distancered <=mindistance) {
            return true;
        }
        if (!isRed && targetid == 5 && distanceblue <=mindistance ) {
            return true;
        }
        return false;
    }
}
