package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;


public class kinematics {
    // set the robot width and length here so that the swerve modules are appropriately placed on the model
    static double RobotLength = 10;
    static double RobotWidth = 10;
    /**
     * Takes in the desired chassis speeds and returns a list of swerve module states to achieve the desired
     * chassis motions
     * @param forward desired forward motion
     * @param left desired motion to the left (negative for right motion)
     * @param turn desired turn
     * @return A list of swerve drive kinematics values to apply to swerve modules
     */
    public static SwerveModuleState[] getSwerveValues(double forward, double left, double turn){
        Translation2d FrontLeft = new Translation2d(RobotLength/2 ,RobotWidth/2);
        Translation2d FrontRight = new Translation2d(RobotLength/2,RobotWidth/-2);
        Translation2d BackLeft = new Translation2d(RobotLength/-2,RobotWidth/2);
        Translation2d BackRight = new Translation2d(RobotLength/-2,RobotWidth/-2);

        SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(FrontLeft, FrontRight, BackLeft, BackRight);

        ChassisSpeeds speeds = new ChassisSpeeds(forward, left, turn);
        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

        return moduleStates;
    }
}
