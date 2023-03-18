package frc.robot;

import frc.robot.util.stateMachine.StateContext;

import java.sql.Time;
import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.auto.Waypoints;
import frc.robot.constants.CameraConstants;
import frc.robot.util.NtHelper;
import frc.robot.util.stateMachine.State;
import frc.robot.util.stateMachine.StateMachine;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation2d;

public class AutoScoreCube extends StateMachine {
    // dw jacob we made it specific just fpr you
    public AutoScoreCube() {
        super("createPath"); // so rad
    }

    public boolean scoringTag() {
        var m_Camera = Robot.m_camera;
        var res = m_Camera.returnCamera().getLatestResult();
        var bestTarget = res.getBestTarget();
        var isRed = Alliance.Red.equals(DriverStation.getAlliance());

        if (res.hasTargets()) {
            var targetid = bestTarget.getFiducialId();

            if (isRed && (targetid == 1 || targetid == 2 || targetid == 3)) {
                return true;
            } else if (!isRed && (targetid == 4 || targetid == 5 || targetid == 6)) {
                return true; // jejwfnewon
            }
        }

        return false;

        // don't want to go to the wrong april tag now do we
    }

    @State(name = "createPath")
    public void createPath(StateContext ctx) {
        if (scoringTag()) {
            var m_Camera = Robot.m_camera;
            var res = m_Camera.returnCamera().getLatestResult();
            var bestTarget = res.getBestTarget();
            var isRed = Alliance.Red.equals(DriverStation.getAlliance());
            var targetID = bestTarget.getFiducialId();
            NtHelper.setDouble("/robot/drivetrain/APRIL_TAG_ID", targetID);
            PathConstraints constraints = new PathConstraints(1.69, 1.69); //Nice^2
            List<PathPoint> waypoints = new ArrayList<>();

            Pose2d start = Robot.m_drive.getPose();
            Pose2d end = Waypoints.aprilTagsScorePoses.get(targetID);

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
        if (Robot.trajectories.isFinishedWithoutTime()) {
            setState("stahp");
        }
        // dew it
    }

    @State(name = "stahp")
    public void stahp(StateContext ctx) {
        Robot.m_drive.drive(0, 0, 0, false);
        setState("scahr");
        // when stop :P
    }

    @State(name = "scahr")
    public void scahr(StateContext ctx) {
        Robot.arm.setShoulderSetpoint(new Rotation2d(Units.degreesToRadians(52)));
        Robot.arm.telescopeToSetpoint(0);
        if (ctx.getTime() > 0.69){  //noice
        Robot.arm.outtakeBelt();
        }
        // cahoobz
    }

    // public void sleep() {
    // //catch some Zs
    // }

}
