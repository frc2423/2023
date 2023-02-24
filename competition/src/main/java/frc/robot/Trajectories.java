package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.NtHelper;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import java.util.List;

public class Trajectories {
    private Trajectory next_path = null;
    private String name = null; // "competition_just_drive_out"
    private List<PathPlannerTrajectory> move_steps;// = PathPlanner.loadPathGroup(name, new PathConstraints(1.5,
                                                   // 3));//2.5, 3
    private final PPHolonomicDriveController m_holonomicController = new PPHolonomicDriveController(
            // feedback in Swerve Module
            new PIDController(1.8, 0, 0), // x feedback
            new PIDController(1.8, 0, 0), // y feedback
            new PIDController(1.8, 0, 0) // w feedback
    );
    private Drivetrain drivetrain = Robot.m_drive;
    private Timer timer = new Timer();

    public void update_current_path() {
        // PathPlanner.loadPath(name, null, false)
        next_path = move_steps.get(0);
        move_steps.remove(0);
        timer.reset();
        timer.start();
    }

    public void resetOdometry(){
        drivetrain.resetOdometry(next_path.getInitialPose());
    }

    public void setNewTrajectoryGroup(String newName) {
        setNewTrajectoryGroup(newName, false);
    }

    public void setNewTrajectoryGroup(String newName, boolean isReversed) {
        name = newName;
        move_steps = PathPlanner.loadPathGroup(name, 2.269, 3, isReversed);// 2.5, 3
        update_current_path();
        Robot.field.getObject("trajectory").setTrajectory(getTrajectory());
    }

    public Trajectory getTrajectory() {
        return next_path;
    }

    public void follow_current_path() {
        drivetrain.updateOdometry();
        var currTime = timer.get();
        System.out.println(timer.get());
        var desiredState = next_path.sample(currTime);
        Robot.field.getObject("ghost").setPose(desiredState.poseMeters);
        // System.out.println(desiredState.poseMeters);
        NtHelper.setDouble("/auto/desiredX", desiredState.poseMeters.getX());
        NtHelper.setDouble("/auto/desiredY", desiredState.poseMeters.getY());
        NtHelper.setDouble("/auto/actualX", drivetrain.getPose().getX());
        NtHelper.setDouble("/auto/actualY", drivetrain.getPose().getY());
        ChassisSpeeds refChassisSpeeds = m_holonomicController.calculate(drivetrain.getPose(),
                (PathPlannerState) desiredState);
        // double vy = RobotBase.isSimulation() ? -refChassisSpeeds.vyMetersPerSecond
        //         : -refChassisSpeeds.vyMetersPerSecond; // if not sim, negetive?

        // double radiansPerSecond = (RobotBase.isSimulation() ? -1 : 1) * refChassisSpeeds.omegaRadiansPerSecond;

        NtHelper.setDouble("/auto/vx", refChassisSpeeds.vxMetersPerSecond);
        NtHelper.setDouble("/auto/vy", refChassisSpeeds.vyMetersPerSecond);
        drivetrain.drive(refChassisSpeeds.vxMetersPerSecond, refChassisSpeeds.vyMetersPerSecond,
                refChassisSpeeds.omegaRadiansPerSecond, false);
    }

    public Boolean isFinished() {
        if (timer.get() > next_path.getTotalTimeSeconds()) {
            return true;
        } else {
            return false;
        }
    }

}
