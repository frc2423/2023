package frc.robot;
 // so rad
import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.auto.Waypoints;
import frc.robot.util.NtHelper;
import frc.robot.util.PhotonRunnable;
import frc.robot.util.stateMachine.State;
import frc.robot.util.stateMachine.StateContext;
import frc.robot.util.stateMachine.StateMachine;

public class AutoHuman extends StateMachine { //Autoo Hooman

    public AutoHuman() {
        super("look");
        //NtHelper.setString("/dashboard/autoScorePosition", "mid");
    }

    public Pose2d transformRedPose(Pose2d startPose) {
        if (Alliance.Red.equals(DriverStation.getAlliance())) {
            double realX = PhotonRunnable.FIELD_LENGTH_METERS - startPose.getX();
            double realY = PhotonRunnable.FIELD_WIDTH_METERS - startPose.getY();
            Rotation2d realANGLE = startPose.getRotation().plus(new Rotation2d(Math.PI));
            Pose2d transformedPose = new Pose2d(realX, realY, realANGLE);
            return transformedPose;
        } else {
            return startPose;
        }
    }

    public boolean scoringTag() {
        var targetid = Robot.photonEstimator.grabBestID();
        
        var isRed = Alliance.Red.equals(DriverStation.getAlliance());

        if (targetid != null) {
           

            if (isRed && ( targetid == 5)) {
                return true;
            } else if (!isRed && (targetid == 4)) {
                return true; // jejwfnewon
            }
        }

        return false;

        // don't want to go to the wrong april tag now do we
    }

    
    @State(name = "look")
    public void createPath(StateContext ctx) {
        if (scoringTag()) {
            var targetID = Robot.photonEstimator.grabBestID();
            
            var isRed = Alliance.Red.equals(DriverStation.getAlliance());
            //NtHelper.setDouble("/robot/drivetrain/APRIL_TAG_ID", targetID);
            PathConstraints constraints = new PathConstraints(1.69, 1.69); //Nice^2
            List<PathPoint> waypoints = new ArrayList<>();

            Pose2d start = Robot.m_drive.getPose();
            Pose2d end = transformRedPose(Waypoints.aprilTagsHumanPlayerStationLongName.get(targetID));

            waypoints.add(new PathPoint(start.getTranslation(), start.getRotation(), start.getRotation()));
            waypoints.add(new PathPoint(end.getTranslation(), end.getRotation(), end.getRotation()));
            Trajectory trajectory = PathPlanner.generatePath(constraints, false, waypoints);
            Robot.trajectories.setNewTrajectoryGroup(trajectory);
            // Robot.trajectories.setNewTrajectoryGroup(Robot.m_drive.getPose(),
            //         Waypoints.aprilTagsScorePoses.get(targetID), false);
            setState("followPath"); //sus
        }
        // staharting @ current pose ending @ score pose
    }

    @State(name = "followPath") // what?!
    public void followPath(StateContext ctx) {
        Robot.trajectories.follow_current_path();
        if (Robot.trajectories.isFinishedWithoutTime(.05 , .05 , 3)) {
            setState("stahp");
        }
        // dew it
    }

    @State(name = "stahp")
    public void stahp(StateContext ctx) {
        Robot.m_drive.drive(0, 0, 0, false);
        Robot.m_drive.setBrake(true);
        // when stop :P
        if (ctx.getTime() > 0.420) { //hehe
            NtHelper.setDouble("/dashboard/armSetpoint/buttonselected", 21);
        }   
        
        
    }

    
}
    
//     @State(name = "mooofawad")
//     public void mooofawad(StateContext ctx) {
//         // Robot.m_drive.drive(0, 0.2, 0, false);
//         PathConstraints constraints = new PathConstraints(1.69, 1.69); //Nice^2
//         List<PathPoint> waypoints = new ArrayList<>();
//         Pose2d start = Robot.m_drive.getPose();
//         Pose2d end = start.plus(new Transform2d(new Translation2d(Units.inchesToMeters(10.69), 0) ,new Rotation2d(0)));//noice //X = 7.69
//        waypoints.add(new PathPoint(start.getTranslation(), start.getRotation(), start.getRotation()));
//        waypoints.add(new PathPoint(end.getTranslation(), new Rotation2d(0), end.getRotation()));
//        Trajectory trajectory = PathPlanner.generatePath(constraints, false, waypoints);
//         Robot.trajectories.setNewTrajectoryGroup(trajectory);
//         setState("GOO");
//     }

//     @State(name = "beSWERVY")
//     public void beSWERVY(StateContext ctx) {
//         SwerveModuleState jilly = new SwerveModuleState(0,
//                     Rotation2d.fromDegrees(0));

//             Robot.m_drive.m_frontLeft.setDesiredState(jilly);
//             Robot.m_drive.m_frontRight.setDesiredState(jilly);
//             Robot.m_drive.m_backLeft.setDesiredState(jilly);
//             Robot.m_drive.m_backRight.setDesiredState(jilly);
//     }
        
//     @State(name = "GOO")
//     public void GOO(StateContext ctx) {
//         Robot.trajectories.follow_current_path();
//         if (ctx.getTime() > .8) {//noice
//             setState("scahr");
//         }
//     }

    
// }
