package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

public class TrajectoryGeneration {
    private static TrajectoryConfig config = new TrajectoryConfig(2, 1);

    public static Trajectory generate(Pose2d start, Pose2d end, boolean isReversed){
        config.setReversed(isReversed);
        List<Translation2d> waypoints = new ArrayList<>();
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(start, waypoints, end, config);
        return trajectory;
    }

    public static Trajectory generate(Pose2d start, Pose2d middle, Pose2d end, boolean isReversed){
        config.setReversed(isReversed);
        List<Translation2d> waypoints = new ArrayList<>();
        waypoints.add(middle.getTranslation());
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(start, waypoints, end, config);
        return trajectory;
    }
}
