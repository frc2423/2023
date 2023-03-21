package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.MathUtil;
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
import frc.robot.util.LinearScale;
import frc.robot.util.NtHelper;
import frc.robot.util.stateMachine.State;
import frc.robot.util.stateMachine.StateContext;
import frc.robot.util.stateMachine.StateMachine;

public class AutoScoreCube extends StateMachine {
    private final LinearScale robotSpin = new LinearScale(Rotation2d.fromDegrees(25).getRadians(), Math.PI,
            Rotation2d.fromDegrees(5).getRadians(),
            Rotation2d.fromDegrees(50).getRadians());
    // dw jacob we made it specific just for you
    public AutoScoreCube() {
        super("createPath"); 
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
            } else if (!isRed && (targetid == 8 || targetid == 7 || targetid == 6)) {
                return true; 
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
            PathConstraints constraints = new PathConstraints(1.69, 1.69); 
            List<PathPoint> waypoints = new ArrayList<>();

            Pose2d start = Robot.m_drive.getPose();
            Pose2d end = Waypoints.aprilTagsScorePoses.get(targetID);

            waypoints.add(new PathPoint(start.getTranslation(), start.getRotation(), start.getRotation()));
            waypoints.add(new PathPoint(end.getTranslation(), end.getRotation(), end.getRotation()));
            Trajectory trajectory = PathPlanner.generatePath(constraints, false, waypoints);
            Robot.trajectories.setNewTrajectoryGroup(trajectory);
            // Robot.trajectories.setNewTrajectoryGroup(Robot.m_drive.getPose(),
            //         Waypoints.aprilTagsScorePoses.get(targetID), false);
            setState("followPath"); 
        }
        // staharting @ current pose ending @ score pose
    }

    @State(name = "followPath") 
    public void followPath(StateContext ctx) {
        Robot.trajectories.follow_current_path();
        if (Robot.trajectories.isFinishedWithoutTime()) {
            setState("finishPath"); 
        }
    }

    @State(name = "finishPath")
    public void finishPath(StateContext ctx) {
        Robot.m_drive.drive(0, 0, 0, false);
        setState("moveForward");
    }

    @State(name = "moveForward")
    public void moveForward(StateContext ctx) {
        // Robot.m_drive.drive(0, 0.2, 0, false);
        PathConstraints constraints = new PathConstraints(1.69, 1.69);
        List<PathPoint> waypoints = new ArrayList<>();
        Pose2d start = Robot.m_drive.getPose();
        Pose2d end = start.plus(new Transform2d(new Translation2d(Units.inchesToMeters(7.69), 0) ,new Rotation2d(0))); //X = 7.69
       waypoints.add(new PathPoint(start.getTranslation(), start.getRotation(), start.getRotation()));
       waypoints.add(new PathPoint(end.getTranslation(), new Rotation2d(0), end.getRotation()));
       Trajectory trajectory = PathPlanner.generatePath(constraints, false, waypoints);
        Robot.trajectories.setNewTrajectoryGroup(trajectory);
        // // var angleError = MathUtil.angleModulus(0 - Robot.m_drive.getOdometryAngle().getRadians());
        // // var rotationSpeed = robotSpin.calculate(angleError);
        // // Robot.m_drive.drive(-0.5, 0, rotationSpeed, true);
        // if (ctx.getTime() > 1.5) {
        // Robot.m_drive.drive(0, 0, 0, true);
            setState("scoreRobot");
        // }

    }

    @State(name = "setSwerveStateZero")
    public void setSwerveStateZero(StateContext ctx) {
        SwerveModuleState jilly = new SwerveModuleState(0,
                    Rotation2d.fromDegrees(0));

            Robot.m_drive.m_frontLeft.setDesiredState(jilly);
            Robot.m_drive.m_frontRight.setDesiredState(jilly);
            Robot.m_drive.m_backLeft.setDesiredState(jilly);
            Robot.m_drive.m_backRight.setDesiredState(jilly);
    }
        
    @State(name = "followRobotPath")
    public void followRobotPath(StateContext ctx) {
        Robot.trajectories.follow_current_path();
        if (ctx.getTime() > 0.69) {
            setState("scoreRobot");
        }
    }

    @State(name = "scoreRobot")
    public void scoreRobot(StateContext ctx) {
        Robot.arm.setShoulderSetpoint(new Rotation2d(Units.degreesToRadians(52)));
        Robot.arm.telescopeToSetpoint(0);
        if (ctx.getTime() > 0.69){  
        Robot.arm.outtakeBelt();
        }
    }

    // public void sleep() {
    // //catch some Zs
    // }

}
