package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class SetPointsButBetter {
    public final static double SHOULDER_LOW_CONE_ANGLE = 32.6;
    public final static double SHOULDER_LOW_CUBE_ANGLE = 32.6;
    public final static double SHOULDER_MID_CONE_ANGLE = 52.6;
    //public final static double SHOULDER_BACK_MID_CONE_ANGLE = new Rotation2d(Units.degreesToRadians(-52.6));
    public final static double SHOULDER_MID_CUBE_ANGLE = 57;
    //public final static double SHOULDER_BACK_MID_CUBE_ANGLE = new Rotation2d(Units.degreesToRadians(-57));
    public final static double SHOULDER_HIGH_CONE_ANGLE = 44.3;
    //public final static double SHOULDER_BACK_HIGH_CONE_ANGLE = new Rotation2d(Units.degreesToRadians(-44.3));
    public final static double SHOULDER_HIGH_CUBE_ANGLE = 58;
    //public final static double SHOULDER_BACK_HIGH_CUBE_ANGLE = new Rotation2d(Units.degreesToRadians(-58));
    public final static double SHOULDERT_DUNK_ANGLE = 80;
    //public final static double SHOULDER_BACK_DUNK_ANGLE = new Rotation2d(Units.degreesToRadians(-80));
    public final static double SHOULDER_HP_ANLGE = 74;
    //public final static double SHOULDER_BACK_HP_ANLGE = new Rotation2d(Units.degreesToRadians(-74));

    public final static Rotation2d SHOULDER_UP_ANGLE = new Rotation2d();
    public final static double SHOULDER_FLOOR_ANGLE = 122;
    //public final static Rotation2d SHOULDER_BACK_FLOOR_ANGLE = new Rotation2d(Units.degreesToRadians(-119));


    public final static double TELESCOPE_FRONT_FLOOR_LENGTH = 0;
    //public final static double TELESCOPE_BACK_FLOOR_LENGTH = 0;
    public final static double TELESCOPE_LOW_CONE_LENGTH = 36;
    public final static double TELESCOPE_LOW_CUBE_LENGTH = 36;
    public final static double TELESCOPE_UP_LENGTH = 0;
    public final static double TELESCOPE_MID_CONE_LENGTH = 20;
    public final static double TELESCOPE_MID_CUBE_LENGTH = 0;
    public final static double TELESCOPE_HIGH_CONE_LENGTH = 30;
    public final static double TELESCOPE_HIGH_CUBE_LENGTH = 23;
    public final static double TELESCOPE_HP_LENGTH = 65;

    public final static double WRIST_FLOOR_ANGLE = 0;
    public final static double WRIST_LOW_CONE_ANGLE = 36;
    public final static double WRIST_LOW_CUBE_ANGLE = 36;
    public final static double WRIST_UP_ANGLE = 0;
    public final static double WRIST_MID_CONE_ANGLE = 20;
    public final static double WRIST_MID_CUBE_ANGLE = 0;
    public final static double WRIST_HIGH_CONE_ANGLE = 30;
    public final static double WRIST_HIGH_CUBE_ANGLE = 23;
    public final static double WRIST_HP_ANGLE = 23;
    
    static class ARM {
        public static final int UP = 5;
        public static final int FRONT_MID = 2;
        public static final int BACK_MID = 8;
        public static final int FRONT_HIGH = 3;
        public static final int BACK_HIGH = 7;
        public static final int FRONT_FLOOR = 1;
        public static final int BACK_FLOOR = 9;
    }
}
