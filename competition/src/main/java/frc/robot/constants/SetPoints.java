package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;


public class SetPoints {
    // public final static Rotation2d SHOULDER_UP_ANGLE = new Rotation2d(Units.degreesToRadians(5));
    // public final static Rotation2d SHOULDER_FRONT_MID_ANGLE = new Rotation2d(Units.degreesToRadians(57));
    // public final static Rotation2d SHOULDER_BACK_MID_ANGLE = new Rotation2d(Units.degreesToRadians(-57));
    // public final static Rotation2d SHOULDER_FRONT_HIGH_ANGLE = new Rotation2d(Units.degreesToRadians(57));
    // public final static Rotation2d SHOULDER_BACK_HIGH_ANGLE = new Rotation2d(Units.degreesToRadians(-57));
    // public final static Rotation2d SHOULDER_FRONT_FLOOR_ANGLE = new Rotation2d(Units.degreesToRadians(115));
    // public final static Rotation2d SHOULDER_BACK_FLOOR_ANGLE = new Rotation2d(Units.degreesToRadians(-115));

    public final static Rotation2d SHOULDER_FRONT_MID_CONE_ANGLE = new Rotation2d(Units.degreesToRadians(53));
    public final static Rotation2d SHOULDER_BACK_MID_CONE_ANGLE = new Rotation2d(Units.degreesToRadians(-53));
    public final static Rotation2d SHOULDER_FRONT_MID_CUBE_ANGLE = new Rotation2d(Units.degreesToRadians(57));
    public final static Rotation2d SHOULDER_BACK_MID_CUBE_ANGLE = new Rotation2d(Units.degreesToRadians(-57));
    public final static Rotation2d SHOULDER_FRONT_HIGH_CONE_ANGLE = new Rotation2d(Units.degreesToRadians(45));
    public final static Rotation2d SHOULDER_BACK_HIGH_CONE_ANGLE = new Rotation2d(Units.degreesToRadians(-45));
    public final static Rotation2d SHOULDER_FRONT_HIGH_CUBE_ANGLE = new Rotation2d(Units.degreesToRadians(58));
    public final static Rotation2d SHOULDER_Back_HIGH_CUBE_ANGLE = new Rotation2d(Units.degreesToRadians(-58));
    public final static Rotation2d SHOULDER_FRONT_DUNK_ANGLE = new Rotation2d(Units.degreesToRadians(67));
    public final static Rotation2d SHOULDER_BACK_DUNK_ANGLE = new Rotation2d(Units.degreesToRadians(-67));


    // public final static double TELESCOPE_UP_LENGTH = 0;
    // public final static double TELESCOPE_FRONT_MID_LENGTH = 20;
    // public final static double TELESCOPE_BACK_MID_LENGTH = 20;
    // public final static double TELESCOPE_FRONT_HIGH_LENGTH = 43;
    // public final static double TELESCOPE_BACK_HIGH_LENGTH = 43;
    // public final static double TELESCOPE_FRONT_FLOOR_LENGTH = 0;
    // public final static double TELESCOPE_BACK_FLOOR_LENGTH = 0;

    public final static double TELESCOPE_UP_LENGTH = 0;
    public final static double TELESCOPE_FRONT_MID_CONE_LENGTH = 21;
    public final static double TELESCOPE_BACK_MID_CONE_LENGTH = 21;
    public final static double TELESCOPE_FRONT_MID_CUBE_LENGTH = 20;
    public final static double TELESCOPE_BACK_MID_CUBE_LENGTH = 20;
    public final static double TELESCOPE_FRONT_HIGH_CONE_LENGTH = 30;
    public final static double TELESCOPE_BACK_HIGH_CONE_LENGTH = 30;
    public final static double TELESCOPE_FRONT_HIGH_CUBE_LENGTH = 24;
    public final static double TELESCOPE_BACK_HIGH_CUBE_LENGTH = 24;



    public static class ARM {
        public static final int UP = 5;
        public static final int FRONT_MID = 2;
        public static final int BACK_MID = 8;
        public static final int FRONT_HIGH = 3;
        public static final int BACK_HIGH = 7;
        public static final int FRONT_FLOOR = 1;
        public static final int BACK_FLOOR = 9;
    }
}
