package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

/** Represents a swerve drive style drivetrain. */
public class PoseEstimation {
  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  private final AnalogGyro m_gyro = new AnalogGyro(0);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  /*
   * Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings.
   * The numbers used
   * below are robot specific, and should be tuned.
   */
  public SwerveModulePosition[] placeholderposition = new SwerveModulePosition[] {
      new SwerveModulePosition(0, new Rotation2d()),
      new SwerveModulePosition(0, new Rotation2d()),
      new SwerveModulePosition(0, new Rotation2d()),
      new SwerveModulePosition(0, new Rotation2d()),
  };
  private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
      m_kinematics,
      m_gyro.getRotation2d(),

      new SwerveModulePosition[] {
          new SwerveModulePosition(0, new Rotation2d()),
          new SwerveModulePosition(0, new Rotation2d()),
          new SwerveModulePosition(0, new Rotation2d()),
          new SwerveModulePosition(0, new Rotation2d()),
      },
      new Pose2d(),
      VecBuilder.fill(1000, 1000, Units.degreesToRadians(5)),
      VecBuilder.fill(0.00001, 0.00001, Units.degreesToRadians(30)));

  public void updateOdometry(PhotonCamera cam) {
    m_poseEstimator.update(new Rotation2d(0, 0), placeholderposition);

    var res = cam.getLatestResult();
    var targetlist = res.getTargets();
    for (int i = 0; i < targetlist.size(); i++) {
      var target = targetlist.get(i);
      var targetid = target.getFiducialId();

      var camToTargetTrans = target.getBestCameraToTarget();

      var camPose = Constants.aprilTags.get(targetid).transformBy(camToTargetTrans.inverse());
      var imageCaptureTime = res.getTimestampSeconds();

      var measurement = camPose.transformBy(Constants.cameraToRobot).toPose2d();
      m_poseEstimator.addVisionMeasurement(
          measurement, imageCaptureTime);
      // m_poseEstimator.
      var position = m_poseEstimator.getEstimatedPosition();
      
      System.out.println("robot pose: (" + position.getX() + ", " + position.getY() + ")");
    }
  }

}
