package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.util.Map;

public class Waypoints {
        // contains all the waypoints that do the waypoint things
        private static final double xBlueGrid = 2.07;
        private static final double xRedGrid = 14.52;

        private static final Rotation2d blueGridRot = new Rotation2d(0);
        private static final Rotation2d redGridRot = Rotation2d.fromDegrees(180);

        private static final double xBlueGP = 6.7;
        private static final double xRedGP = 10.46;

        // positions
        public static final Pose2d BLUE_GRID_1 = new Pose2d(xBlueGrid, 0.48, blueGridRot); // (boring)
        public static final Pose2d BLUE_GRID_2 = new Pose2d(xBlueGrid, 1.05, blueGridRot);
        public static final Pose2d BLUE_GRID_3 = new Pose2d(xBlueGrid, 1.60, blueGridRot);
        public static final Pose2d BLUE_GRID_4 = new Pose2d(xBlueGrid, 2.20, blueGridRot);
        public static final Pose2d BLUE_GRID_5 = new Pose2d(xBlueGrid, 2.75, blueGridRot);
        public static final Pose2d BLUE_GRID_6 = new Pose2d(xBlueGrid, 3.30, blueGridRot);
        public static final Pose2d BLUE_GRID_7 = new Pose2d(xBlueGrid, 3.85, blueGridRot);
        public static final Pose2d BLUE_GRID_8 = new Pose2d(xBlueGrid, 4.40, blueGridRot);
        public static final Pose2d BLUE_GRID_9 = new Pose2d(xBlueGrid, 4.97, blueGridRot);

        public static final Pose2d BLUE_GP_1 = new Pose2d(xBlueGP, .93, new Rotation2d());
        public static final Pose2d BLUE_GP_2 = new Pose2d(xBlueGP, 1.74, Rotation2d.fromDegrees(28.64));
        public static final Pose2d BLUE_GP_3 = new Pose2d(xBlueGP, 3.93, Rotation2d.fromDegrees(-29.29));
        public static final Pose2d BLUE_GP_4 = new Pose2d(xBlueGP, 4.62, new Rotation2d());

        public static final Pose2d BLUE_CHARGE_1 = new Pose2d(3.92, 0.79, new Rotation2d());
        public static final Pose2d BLUE_CHARGE_2 = new Pose2d(3.92, 4.76, new Rotation2d());

        public static final Pose2d RED_GRID_1 = new Pose2d(xRedGrid, 0.48, redGridRot);
        public static final Pose2d RED_GRID_2 = new Pose2d(xRedGrid, 1.05, redGridRot);
        public static final Pose2d RED_GRID_3 = new Pose2d(xRedGrid, 1.60, redGridRot);
        public static final Pose2d RED_GRID_4 = new Pose2d(xRedGrid, 2.20, redGridRot);
        public static final Pose2d RED_GRID_5 = new Pose2d(xRedGrid, 2.75, redGridRot);
        public static final Pose2d RED_GRID_6 = new Pose2d(xRedGrid, 3.30, redGridRot);
        public static final Pose2d RED_GRID_7 = new Pose2d(xRedGrid, 3.85, redGridRot);
        public static final Pose2d RED_GRID_8 = new Pose2d(xRedGrid, 4.40, redGridRot);
        public static final Pose2d RED_GRID_9 = new Pose2d(xRedGrid, 4.97, redGridRot);

        public static final Pose2d RED_GP_1 = new Pose2d(xRedGP, 0.96, Rotation2d.fromDegrees(180));
        public static final Pose2d RED_GP_2 = new Pose2d(xRedGP, 1.74, Rotation2d.fromDegrees(155.11));
        public static final Pose2d RED_GP_3 = new Pose2d(xRedGP, 3.93, Rotation2d.fromDegrees(-163.74));
        public static final Pose2d RED_GP_4 = new Pose2d(xRedGP, 4.62, Rotation2d.fromDegrees(180));

        public static final Pose2d RED_CHARGE_1 = new Pose2d(12.62, 0.79, Rotation2d.fromDegrees(180));
        public static final Pose2d RED_CHARGE_2 = new Pose2d(12.62, 4.76, Rotation2d.fromDegrees(180));

        public static final Pose2d RED_HP = new Pose2d(12.62, 0.79, Rotation2d.fromDegrees(180)); //figure it out man
        public static final Pose2d BLUE_HP = new Pose2d(13.84, 7.27, Rotation2d.fromDegrees(90));
        // april tag section

        // public static final Pose2d BLUE_AT_2 = BLUE_GRID_5;
        public static final Map<Integer, Pose2d> aprilTagsScorePosesCubes = // Poses Moses
                Map.of(
                                1,
                                RED_GRID_2.plus(new Transform2d(new Translation2d(Units.inchesToMeters(4), 0),
                                                new Rotation2d(Math.PI))),
                                2,
                                RED_GRID_5.plus(new Transform2d(new Translation2d(Units.inchesToMeters(4), 0),
                                                new Rotation2d(Math.PI))),
                                3,
                                RED_GRID_8.plus(new Transform2d(new Translation2d(Units.inchesToMeters(4), Units.inchesToMeters(-2)),
                                                new Rotation2d(Math.PI))), // this is changed for cones
                                6,
                                BLUE_GRID_8.plus(new Transform2d(new Translation2d(Units.inchesToMeters(-4), 0),
                                                new Rotation2d(Math.PI))),
                                7,
                                BLUE_GRID_5.plus(new Transform2d(new Translation2d(Units.inchesToMeters(-4), 0),
                                                new Rotation2d(Math.PI))),
                                8,
                                BLUE_GRID_2.plus(new Transform2d(new Translation2d(Units.inchesToMeters(-4), 0),
                                                new Rotation2d(Math.PI))));

        public static final Map<Integer, Pose2d> aprilTagsScorePosesConesLeft = // Poses Moses
                Map.of(
                                1,
                                RED_GRID_3.plus(new Transform2d(new Translation2d(Units.inchesToMeters(4), 0),
                                                new Rotation2d(Math.PI))),
                                2,
                                RED_GRID_6.plus(new Transform2d(new Translation2d(Units.inchesToMeters(4), 0),
                                                new Rotation2d(Math.PI))),
                                3,
                                RED_GRID_9.plus(new Transform2d(new Translation2d(Units.inchesToMeters(4), 0),
                                                new Rotation2d(Math.PI))),
                                6,
                                BLUE_GRID_7.plus(new Transform2d(new Translation2d(Units.inchesToMeters(-4 + 3), 0),
                                                new Rotation2d(Math.PI))),
                                7,
                                BLUE_GRID_4.plus(new Transform2d(new Translation2d(Units.inchesToMeters(-4 + 3), 0),
                                                new Rotation2d(Math.PI))),
                                8,
                                BLUE_GRID_1.plus(new Transform2d(new Translation2d(Units.inchesToMeters(-4 + 3), 0),
                                                new Rotation2d(Math.PI))));

                

        public static final Map<Integer, Pose2d> aprilTagsScorePosesConesRight = // Poses Moses
                Map.of(
                                1,
                                RED_GRID_1.plus(new Transform2d(new Translation2d(Units.inchesToMeters(4), 0),
                                                new Rotation2d(Math.PI))),
                                2,
                                RED_GRID_4.plus(new Transform2d(new Translation2d(Units.inchesToMeters(4), 0),
                                                new Rotation2d(Math.PI))),
                                3,
                                RED_GRID_7.plus(new Transform2d(new Translation2d(Units.inchesToMeters(4), 0),
                                                new Rotation2d(Math.PI))),
                                6,
                                BLUE_GRID_9.plus(new Transform2d(new Translation2d(Units.inchesToMeters(-4), 0),
                                                new Rotation2d(Math.PI))),
                                7,
                                BLUE_GRID_6.plus(new Transform2d(new Translation2d(Units.inchesToMeters(-4), 0),
                                                new Rotation2d(Math.PI))),
                                8,
                                BLUE_GRID_3.plus(new Transform2d(new Translation2d(Units.inchesToMeters(-4), 0),
                                                new Rotation2d(Math.PI))));
                
        public static final Map<Integer, Pose2d> aprilTagsHumanPlayerStationLongName = //Tag 4 = Red, Tag 5 = Blue
                Map.of(
                                4,
                                BLUE_HP, 
                                5, 
                                RED_HP
                );

}
