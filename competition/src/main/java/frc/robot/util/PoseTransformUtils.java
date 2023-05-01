package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class PoseTransformUtils {

    public static final double FIELD_LENGTH_METERS = 16.54175;
    public static final double FIELD_WIDTH_METERS = 8.0137;

    public static boolean isRedAlliance() {
        return Alliance.Red.equals(DriverStation.getAlliance());
    }

    public static Pose2d transformRedPose(Pose2d startPose) {
        if (isRedAlliance()) {
            double realX = FIELD_LENGTH_METERS - startPose.getX();
            double realY = FIELD_WIDTH_METERS - startPose.getY();
            Rotation2d realANGLE = startPose.getRotation().plus(new Rotation2d(Math.PI));
            Pose2d transformedPose = new Pose2d(realX, realY, realANGLE);
            return transformedPose;
        } else {
            return startPose;
        }
    }
}
