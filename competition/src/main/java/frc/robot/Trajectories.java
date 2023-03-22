package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.NtHelper;
import frc.robot.util.TrajectoryGeneration;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
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

    public Trajectories() {
        m_holonomicController.setTolerance(new Pose2d(0.02, 0.02, Rotation2d.fromDegrees(2.5)));
    }

    public void update_current_path() {
        // PathPlanner.loadPath(name, null, false)
        next_path = move_steps.get(0);
        move_steps.remove(0);
        timer.reset();
        timer.start();
    }

    public void resetOdometry() {
        drivetrain.resetOdometry(next_path.getInitialPose());
    }

    public void setNewTrajectoryGroup(String newName) {
        setNewTrajectoryGroup(newName, false);
    }

    public void setNewTrajectoryGroup(Pose2d start, Pose2d end, boolean isDrivingBackwards) {
        next_path = TrajectoryGeneration.generate(start, end, isDrivingBackwards);
        Robot.field.getObject("trajectory").setTrajectory(getTrajectory());
        timer.reset();
        timer.start();
    }

    public void setNewTrajectoryGroup(Pose2d start, Pose2d middle, Pose2d end, boolean isDrivingBackwards) {
        next_path = TrajectoryGeneration.generate(start, middle, end, isDrivingBackwards);
        Robot.field.getObject("trajectory").setTrajectory(getTrajectory());
        timer.reset();
        timer.start();
    }

    public void setNewTrajectoryGroup(String newName, boolean isReversed) {
        name = newName;
        move_steps = PathPlanner.loadPathGroup(name, 2.269, 3, isReversed);// 2.5, 3
        update_current_path();
        Robot.field.getObject("trajectory").setTrajectory(getTrajectory());
    }

    public void setNewTrajectoryGroup(Trajectory jort) {
        next_path = jort;
        Robot.field.getObject("trajectory").setTrajectory(getTrajectory());
        timer.reset();
        timer.start();
    }

    public Trajectory getTrajectory() {
        return next_path;
    }

    public void follow_current_path() {
        // drivetrain.updateOdometry();
        var currTime = timer.get();
        var desiredState = next_path.sample(currTime);
        Robot.field.getObject("ghost/pose").setPose(desiredState.poseMeters);
        NtHelper.setDouble("/SmartDashboard/Field/ghost/opacity", 50);

        ChassisSpeeds refChassisSpeeds = m_holonomicController.calculate(drivetrain.getPose(),
                (PathPlannerState) desiredState);

        drivetrain.drive(refChassisSpeeds.vxMetersPerSecond, refChassisSpeeds.vyMetersPerSecond,
                refChassisSpeeds.omegaRadiansPerSecond, false);
    }

    public Boolean isFinished() {
        var robotPose =  drivetrain.getPose();
        var robotAngle = robotPose.getRotation();
        var robotPosition = robotPose.getTranslation();
        var  robotLastPose = next_path.sample(next_path.getTotalTimeSeconds()).poseMeters;
        var robotLastPos = robotLastPose.getTranslation();
        var robotLastAngle = robotLastPose.getRotation();
        var distFinal = robotLastPos.getDistance(robotPosition);
        var angleDif = robotLastAngle.minus(robotAngle);

        if (timer.get() > next_path.getTotalTimeSeconds() + 1.5 || distFinal < 0.1 && Math.abs(angleDif.getDegrees())< 5

        ) {
        return true;
        } else {
        return false;
        }
        // // return false;
        // // if (m_holonomicController.atReference()) {
        // //     return true;
        // // } else {
        // //     return false;
        // // }
    }

    public Boolean isFinishedWithoutTime() {
        var robotPose =  drivetrain.getPose();
        var robotAngle = robotPose.getRotation();
        var robotPosition = robotPose.getTranslation();
        var  robotLastPose = next_path.sample(next_path.getTotalTimeSeconds()).poseMeters;
        var robotLastPos = robotLastPose.getTranslation();
        var robotLastAngle = robotLastPose.getRotation();
        var distFinal = robotLastPos.getDistance(robotPosition);
        var angleDif = robotLastAngle.minus(robotAngle);

        if ( distFinal < 0.1 && Math.abs(angleDif.getDegrees())< 5

        ) {
        return true;
        } else {
        return false;
        }
        // // return false;
        // // if (m_holonomicController.atReference()) {
        // //     return true;
        // // } else {
        // //     return false;
        // // }
    }
}
