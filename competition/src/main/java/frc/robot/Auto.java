package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import java.util.List;

public class Auto {
    private Trajectory next_path = null;
    private List<PathPlannerTrajectory> move_steps = PathPlanner.loadPathGroup("SimVisible", new PathConstraints(3, 2));
    private final PPHolonomicDriveController m_holonomicController = new PPHolonomicDriveController(
        new PIDController(0, 0, 0), // x feedback
        new PIDController(0, 0, 0), // y feedback
        new PIDController(0, 0, 0) // w feedback
    );
    private Drivetrain drivetrain;
    private Timer timer = new Timer();

    public Auto(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    public void update_current_path() {
        next_path = move_steps.get(0);
        move_steps.remove(0);
        timer.reset();
        timer.start();
    }

    public void follow_current_path() {
        drivetrain.updateOdometry();
        var currTime = timer.get();
        var desiredState = next_path.sample(currTime);
        ChassisSpeeds refChassisSpeeds = m_holonomicController.calculate(drivetrain.getPose(), (PathPlannerState)desiredState);
        System.out.println(refChassisSpeeds);
        drivetrain.drive(refChassisSpeeds.vxMetersPerSecond, refChassisSpeeds.vyMetersPerSecond, refChassisSpeeds.omegaRadiansPerSecond, true);
    }

    public Pose2d getTarget() {
        return new Pose2d(0, 1.7526, new Rotation2d());
    }

    public void robotGo() {
    
    }

}
