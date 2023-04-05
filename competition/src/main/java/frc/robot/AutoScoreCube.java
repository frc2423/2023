package frc.robot;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonUtils;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.auto.Waypoints;
import frc.robot.constants.SetPoints;
import frc.robot.util.NtHelper;
import frc.robot.util.PhotonRunnable;
import frc.robot.util.stateMachine.State;
import frc.robot.util.stateMachine.StateContext;
import frc.robot.util.stateMachine.StateMachine;

public class AutoScoreCube extends StateMachine {
    // dw jacob we made it specific just for you
    public AutoScoreCube() {
        super("createPath"); // so rad
        NtHelper.setString("/dashboard/autoScorePosition", "mid");
    }

    public String getScoringTagLabel() {
        var targetid = Robot.photonEstimator.grabBestID();

        if (targetid == null) {
            return "None";
        } 

        if (targetid == 1) {
            return "RED Left <--";
        }
        else if (targetid == 2) {
            return "RED Middle ^";
        }
        else if (targetid == 3) {
            return "RED Right -->";
        }
        else if (targetid == 4) {
            return "BLUE HP";
        }
        else if (targetid == 5) {
            return "RED HP";
        }
        else if (targetid == 6) {
            return "BLUE Left <--";
        }
        else if (targetid == 7) {
            return "BLUE Middle ^";
        }
        else if (targetid == 8) {
            return "BLUE Right -->";
        }
        else {
            return "Unknown";
        }
    

    }

    public boolean scoringTag() {
        var targetid = Robot.photonEstimator.grabBestID();

        var isRed = Alliance.Red.equals(DriverStation.getAlliance());

        if (targetid != null) {

            if (isRed && (targetid == 1 || targetid == 2 || targetid == 3)) {
                return true;
            } else if (!isRed && (targetid == 8 || targetid == 7 || targetid == 6)) {
                return true; // jejwfnewon
            }
        }

        return false;

        // don't want to go to the wrong april tag now do we
    }

    // private double getClosestAprilTag() {
    //     var targetid = Robot.photonEstimator.grabBestID();

    //     if (targets.size() > 1) {
    //         double smallestdist = 10000000;
    //         int bestid = 0;
    //         for (int i = 0; i < targets.size(); i++) {
    //             var target = targets.get(i);
    //             var id = target.getFiducialId();
    //             var pose = Waypoints.aprilTagsScorePosesCubes.get(id);
    //             var distance = PhotonUtils.getDistanceToPose(pose, Robot.m_drive.getPose());
    //             if (distance < smallestdist) {
    //                 smallestdist = distance;
    //                 bestid = id;

    //             }

    //         }
    //         return bestid;

    //     }
    //     return Robot.m_camera.getLatestResult().getBestTarget().getFiducialId();

    // }

    public Pose2d getIsLeft(double id) {
        Pose2d left = Waypoints.aprilTagsScorePosesConesLeft.get((int) id);
        Pose2d right = Waypoints.aprilTagsScorePosesConesRight.get((int) id);
        String IsLeft = NtHelper.getString("/robot/autoScore/position", "right");
        if (IsLeft.equals("left")) {
            return left;
        } else {
            return right;
        }
    }

    public void setScorePosition() {
        // get value from networktables
        // value is "high", "mid", or "low"
        var position = NtHelper.getString("/dashboard/autoScorePosition", "mid");
        boolean isCubes = NtHelper.getBoolean("/dashboard/arm/isCubes", false);
        if (isCubes) {

            // Set telescope and shoulder to position
            if (position.equals("high")) {
                Robot.arm.setShoulderSetpoint(SetPoints.SHOULDER_FRONT_HIGH_CUBE_ANGLE);
                Robot.arm.telescopeToSetpoint(SetPoints.TELESCOPE_HIGH_CUBE_LENGTH);
            }

            if (position.equals("mid")) {
                Robot.arm.setShoulderSetpoint(SetPoints.SHOULDER_FRONT_MID_CUBE_ANGLE);
                Robot.arm.telescopeToSetpoint(0);
            }

            if (position.equals("low")) {
                Robot.arm.setShoulderSetpoint(SetPoints.SHOULDER_FRONT_FLOOR_ANGLE);
                Robot.arm.telescopeToSetpoint(0);
            }
        } else {
            if (position.equals("high")) {
                Robot.arm.setShoulderSetpoint(SetPoints.SHOULDER_FRONT_HIGH_CONE_ANGLE);
                Robot.arm.telescopeToSetpoint(SetPoints.TELESCOPE_HIGH_CONE_LENGTH);
            }

            if (position.equals("mid")) {
                Robot.arm.setShoulderSetpoint(SetPoints.SHOULDER_FRONT_MID_CONE_ANGLE);
                Robot.arm.telescopeToSetpoint(SetPoints.TELESCOPE_MID_CONE_LENGTH);
            }

            if (position.equals("low")) {
                Robot.arm.setShoulderSetpoint(SetPoints.SHOULDER_FRONT_FLOOR_ANGLE);
                Robot.arm.telescopeToSetpoint(0);
            }
        }
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

    @State(name = "createPath")
    public void createPath(StateContext ctx) {
        if (scoringTag()) {
            var targetid = Robot.photonEstimator.grabBestID();

            var isRed = Alliance.Red.equals(DriverStation.getAlliance());
            NtHelper.setDouble("/robot/drivetrain/APRIL_TAG_ID", targetid);
            PathConstraints constraints = new PathConstraints(1.69, 1.69); // Nice^2
            List<PathPoint> waypoints = new ArrayList<>();

            Pose2d start = Robot.m_drive.getPose();
            int closeApirlTag = targetid;
            Pose2d end;
            if (NtHelper.getBoolean("/dashboard/arm/isCubes", true)) {
                end = transformRedPose(Waypoints.aprilTagsScorePosesCubes.get(closeApirlTag));
            } else {
                end = transformRedPose(getIsLeft(closeApirlTag));
            }

            waypoints.add(new PathPoint(start.getTranslation(), start.getRotation(), start.getRotation()));
            waypoints.add(new PathPoint(end.getTranslation(), end.getRotation(), end.getRotation()));
            Trajectory trajectory = PathPlanner.generatePath(constraints, false, waypoints);
            Robot.trajectories.setNewTrajectoryGroup(trajectory);
            // Robot.trajectories.setNewTrajectoryGroup(Robot.m_drive.getPose(),
            // Waypoints.aprilTagsScorePoses.get(targetID), false);
            setState("followPath"); // sus
        }
        // staharting @ current pose ending @ score pose
    }

    @State(name = "followPath") // what?!
    public void followPath(StateContext ctx) {
        Robot.trajectories.follow_current_path();
        if (Robot.trajectories.isFinishedWithoutTime(.03, .03, 5)) {
            setState("stahp");
        }
        // dew it
    }

    @State(name = "stahp")
    public void stahp(StateContext ctx) {
        Robot.m_drive.drive(0, 0, 0, false);
        // when stop :P
        if (NtHelper.getString("/dashboard/autoScorePosition", "mid").equals("low")) {
            setState("scahr");
        } else {
            setState("mooofawad");
        }
    }

    @State(name = "mooofawad")
    public void mooofawad(StateContext ctx) {
        // Robot.m_drive.drive(0, 0.2, 0, false);
        PathConstraints constraints = new PathConstraints(1.69, 1.69); // Nice^2
        List<PathPoint> waypoints = new ArrayList<>();
        Pose2d start = Robot.m_drive.getPose();
        Pose2d end = transformRedPose(
                start.plus(new Transform2d(new Translation2d(Units.inchesToMeters(5), 0), new Rotation2d(0))));// noice
                                                                                                               // //X =
                                                                                                               // 7.69
        waypoints.add(new PathPoint(start.getTranslation(), start.getRotation(), start.getRotation()));
        waypoints.add(new PathPoint(end.getTranslation(), new Rotation2d(0), end.getRotation()));
        Trajectory trajectory = PathPlanner.generatePath(constraints, false, waypoints);
        Robot.trajectories.setNewTrajectoryGroup(trajectory);
        setState("GOO");
    }

    @State(name = "beSWERVY")
    public void beSWERVY(StateContext ctx) {
        SwerveModuleState jilly = new SwerveModuleState(0,
                Rotation2d.fromDegrees(0));

        Robot.m_drive.m_frontLeft.setDesiredState(jilly);
        Robot.m_drive.m_frontRight.setDesiredState(jilly);
        Robot.m_drive.m_backLeft.setDesiredState(jilly);
        Robot.m_drive.m_backRight.setDesiredState(jilly);
    }

    @State(name = "GOO")
    public void GOO(StateContext ctx) {
        Robot.trajectories.follow_current_path();
        if (ctx.getTime() > .8) {// noice
            setState("scahr");
        }
    }

    @State(name = "scahr")
    public void scahr(StateContext ctx) {
        Robot.m_drive.drive(0, 0, 0, false);
        setScorePosition();
        boolean isCubes = NtHelper.getBoolean("/dashboard/arm/isCubes", false);

        if (isCubes) {

            if (ctx.getTime() > 0.5) {
                if (!isCubes) {
                    Robot.arm.setShoulderSetpoint(SetPoints.SHOULDER_FRONT_DUNK_ANGLE);
                }
            }
            if (ctx.getTime() > 1.35) {
                Robot.arm.outtakeBelt();
            }
        }

        // cahoobz
    }

    // public void sleep() {
    // //catch some Zs
    // }

}
