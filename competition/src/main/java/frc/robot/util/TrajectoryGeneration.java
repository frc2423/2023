package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

public class TrajectoryGeneration {
    static PathConstraints path = new PathConstraints(2, 2);
    public static Trajectory generate(Pose2d start, Pose2d end, boolean isReversed){
        List<PathPoint> waypoints = new ArrayList<>();
        waypoints.add(new PathPoint(start.getTranslation(), start.getRotation(), start.getRotation()));
        waypoints.add(new PathPoint(end.getTranslation(), end.getRotation(), end.getRotation()));
        Trajectory trajectory = PathPlanner.generatePath(path, isReversed, waypoints);
        return trajectory;
    }

    public static Trajectory generate(Pose2d start, Pose2d middle, Pose2d end, boolean isReversed){
        List<PathPoint> waypoints = new ArrayList<>();
        waypoints.add(new PathPoint(start.getTranslation(), start.getRotation(), start.getRotation()));
        waypoints.add(new PathPoint(middle.getTranslation(), middle.getRotation(), middle.getRotation()));
        waypoints.add(new PathPoint(end.getTranslation(), end.getRotation(), end.getRotation()));
        Trajectory trajectory = PathPlanner.generatePath(path, isReversed, waypoints);
        return trajectory;
    }
}
