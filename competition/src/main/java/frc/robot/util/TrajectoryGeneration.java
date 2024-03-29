package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class TrajectoryGeneration {
    static PathConstraints constraints = new PathConstraints(1.5, 1.5);

    public static Trajectory generate(Pose2d start, Pose2d end, boolean isDrivingBackwards) {
        List<PathPoint> waypoints = new ArrayList<>();
        if (isDrivingBackwards) {
            waypoints.add(new PathPoint(start.getTranslation(), start.getRotation().plus(Rotation2d.fromDegrees(-180)),
                    start.getRotation()));
            waypoints.add(new PathPoint(end.getTranslation(), end.getRotation().plus(Rotation2d.fromDegrees(-180)),
                    end.getRotation()));
        } else {
            waypoints.add(new PathPoint(start.getTranslation(), start.getRotation(), start.getRotation()));
            waypoints.add(new PathPoint(end.getTranslation(), end.getRotation(), end.getRotation()));
        }
        Trajectory trajectory = PathPlanner.generatePath(constraints, false, waypoints);
        if (Alliance.Red.equals(DriverStation.getAlliance())) {
            return PathPlannerTrajectory.transformTrajectoryForAlliance((PathPlannerTrajectory) trajectory,
                    DriverStation.getAlliance());
        }
        return trajectory;
    }

    public static Trajectory generate(Pose2d start, Pose2d middle, Pose2d end, boolean isDrivingBackwards) {
        List<PathPoint> waypoints = new ArrayList<>();
        if (isDrivingBackwards) {
            waypoints.add(new PathPoint(start.getTranslation(), start.getRotation().plus(Rotation2d.fromDegrees(-180)), start.getRotation()));
            waypoints.add(new PathPoint(middle.getTranslation(), middle.getRotation().plus(Rotation2d.fromDegrees(-180)), middle.getRotation()));
            waypoints.add(new PathPoint(end.getTranslation(), end.getRotation().plus(Rotation2d.fromDegrees(-180)), end.getRotation()));
        } else {
            waypoints.add(new PathPoint(start.getTranslation(), start.getRotation(), start.getRotation()));
            waypoints.add(new PathPoint(middle.getTranslation(), middle.getRotation(), middle.getRotation()));
            waypoints.add(new PathPoint(end.getTranslation(), end.getRotation(), end.getRotation()));
        }
        Trajectory trajectory = PathPlanner.generatePath(constraints, false, waypoints);
        if (Alliance.Red.equals(DriverStation.getAlliance())) {
            return PathPlannerTrajectory.transformTrajectoryForAlliance((PathPlannerTrajectory) trajectory,
                    DriverStation.getAlliance());
        }
        return trajectory;
    }
}
