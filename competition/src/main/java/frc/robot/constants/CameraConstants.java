package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.Map;

public class CameraConstants {
    /** the accurate map of the field (use for the actual game) */
    public final static double CAMERA_HEIGHT_METERS = Units.inchesToMeters(28);
    public static double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
    public final static double CAMERA_PITCH_RADIANS = Units.degreesToRadians(-12);
    public final static double CAMERA_X_METERS = Units.inchesToMeters(-5);
    public final static double CAMERA_Y_METERS = Units.inchesToMeters(5);

    public static final Map<Integer, Pose3d> aprilTags = 
    Map.of(
        1,
        new Pose3d(
            Units.inchesToMeters(610.77),
            Units.inchesToMeters(42.19),
            Units.inchesToMeters(18.22),
            new Rotation3d(0.0, 0.0, Math.PI /*3.1415926535697932384626433 */)),
        2,
        new Pose3d(
            Units.inchesToMeters(610.77),
            Units.inchesToMeters(108.19),
            Units.inchesToMeters(18.22),
            new Rotation3d(0.0, 0.0, Math.PI)),
        3,
        new Pose3d(
            Units.inchesToMeters(610.77),
            Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
            Units.inchesToMeters(18.22),
            new Rotation3d(0.0, 0.0, Math.PI)),
        4,
        new Pose3d(
            Units.inchesToMeters(636.96),
            Units.inchesToMeters(265.74),
            Units.inchesToMeters(27.38),
            new Rotation3d(0.0, 0.0, Math.PI)),
        5,
        new Pose3d(
            Units.inchesToMeters(14.25),
            Units.inchesToMeters(265.74),
            Units.inchesToMeters(27.38),
            new Rotation3d()),
        6,
        new Pose3d(
            Units.inchesToMeters(40.45),
            Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
            Units.inchesToMeters(18.22),
            new Rotation3d()),
        7,
        new Pose3d(
            Units.inchesToMeters(40.45),
            Units.inchesToMeters(108.19),
            Units.inchesToMeters(18.22),
            new Rotation3d()),
        8,
        new Pose3d(
            Units.inchesToMeters(40.45),
            Units.inchesToMeters(42.19),
            Units.inchesToMeters(18.22),
            new Rotation3d()));
        
        /** testing positions */
        /**
        public static final Map<Integer, Pose3d> aprilTags =
        Map.of(
            1,
            new Pose3d(
                Units.inchesToMeters(1000),
                Units.inchesToMeters(1000),
                Units.inchesToMeters(0),
                new Rotation3d(0.0, 0.0, Math.PI)),
            2,
            new Pose3d(
                Units.inchesToMeters(-1000),
                Units.inchesToMeters(1000),
                Units.inchesToMeters(0),
                new Rotation3d(0.0, 0.0, Math.PI)),
            3,
            new Pose3d(
                Units.inchesToMeters(-1000),
                Units.inchesToMeters(-1000), // FIRST's diagram has a typo (it says 147.19)
                Units.inchesToMeters(18.22),
                new Rotation3d(0.0, 0.0, Math.PI)),
            4,
            new Pose3d(
                Units.inchesToMeters(1000),
                Units.inchesToMeters(-1000),
                Units.inchesToMeters(27.38),
                new Rotation3d(0.0, 0.0, Math.PI)),
            5,
            new Pose3d(
                Units.inchesToMeters(14.25),
                Units.inchesToMeters(265.74),
                Units.inchesToMeters(27.38),
                new Rotation3d()),
            6,
            new Pose3d(
                Units.inchesToMeters(40.45),
                Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
                Units.inchesToMeters(18.22),
                new Rotation3d()),
            7,
            new Pose3d(
                Units.inchesToMeters(40.45),
                Units.inchesToMeters(108.19),
                Units.inchesToMeters(18.22),
                new Rotation3d()),
            8,
            new Pose3d(
                Units.inchesToMeters(40.45),
                Units.inchesToMeters(42.19),
                Units.inchesToMeters(18.22),
                new Rotation3d()));
*/

        public static final Transform3d cameraToRobot = 

        new Transform3d(
            new Translation3d(CAMERA_X_METERS,CAMERA_Y_METERS, CAMERA_HEIGHT_METERS),
            new Rotation3d(0, CAMERA_PITCH_RADIANS, 0)
        );
}

