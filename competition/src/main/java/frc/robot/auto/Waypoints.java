package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class Waypoints {
    //contains all the waypoints that do the waypoint things
    private static final double xBlueGrid = 2.07;
    private static final double xRedGrid = 0;

    private static final Rotation2d blueGridRot = new Rotation2d(0);
    private static final Rotation2d redGridRot = new Rotation2d();

    private static final double xBlueGP = 6.03;
    private static final double xRedGP = 0;

    //positions
    public static final Pose2d BLUE_GRID_1 = new Pose2d(xBlueGrid, 0.48, blueGridRot); //(boring)
    public static final Pose2d BLUE_GRID_2 = new Pose2d(xBlueGrid, 0, blueGridRot);
    public static final Pose2d BLUE_GRID_3 = new Pose2d(xBlueGrid, 0, blueGridRot);
    public static final Pose2d BLUE_GRID_4 = new Pose2d(xBlueGrid, 0, blueGridRot);
    public static final Pose2d BLUE_GRID_5 = new Pose2d(xBlueGrid, 0, blueGridRot);
    public static final Pose2d BLUE_GRID_6 = new Pose2d(xBlueGrid, 0, blueGridRot);
    public static final Pose2d BLUE_GRID_7 = new Pose2d(xBlueGrid, 0, blueGridRot);
    public static final Pose2d BLUE_GRID_8 = new Pose2d(xBlueGrid, 0, blueGridRot);
    public static final Pose2d BLUE_GRID_9 = new Pose2d(xBlueGrid, 0, blueGridRot);

    public static final Pose2d BLUE_GP_1 = new Pose2d(xBlueGP, 0.71, Rotation2d.fromDegrees(14.26)); 
    public static final Pose2d BLUE_GP_2 = new Pose2d(xBlueGP, 0, null);
    public static final Pose2d BLUE_GP_3 = new Pose2d(xBlueGP, 0, null);
    public static final Pose2d BLUE_GP_4 = new Pose2d(xBlueGP, 0, null);

    public static final Pose2d BLUE_CHARGE_1 = new Pose2d(0, 0, null); 
    public static final Pose2d BLUE_CHARGE_2 = new Pose2d(0, 0, null);

    public static final Pose2d RED_GRID_1 = new Pose2d(xRedGrid, 0, redGridRot); 
    public static final Pose2d RED_GRID_2 = new Pose2d(xRedGrid, 0, redGridRot);
    public static final Pose2d RED_GRID_3 = new Pose2d(xRedGrid, 0, redGridRot);
    public static final Pose2d RED_GRID_4 = new Pose2d(xRedGrid, 0, redGridRot);
    public static final Pose2d RED_GRID_5 = new Pose2d(xRedGrid, 0, redGridRot);
    public static final Pose2d RED_GRID_6 = new Pose2d(xRedGrid, 0, redGridRot);
    public static final Pose2d RED_GRID_7 = new Pose2d(xRedGrid, 0, redGridRot);
    public static final Pose2d RED_GRID_8 = new Pose2d(xRedGrid, 0, redGridRot);
    public static final Pose2d RED_GRID_9 = new Pose2d(xRedGrid, 0, redGridRot);

    public static final Pose2d RED_GP_1 = new Pose2d(xRedGP, 0, null); 
    public static final Pose2d RED_GP_2 = new Pose2d(xRedGP, 0, null);
    public static final Pose2d RED_GP_3 = new Pose2d(xRedGP, 0, null);
    public static final Pose2d RED_GP_4 = new Pose2d(xRedGP, 0, null);

    public static final Pose2d RED_CHARGE_1 = new Pose2d(0, 0, null); 
    public static final Pose2d RED_CHARGE_2 = new Pose2d(0, 0, null);
}
