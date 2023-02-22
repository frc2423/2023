// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.NtHelper;

public class Drivetrain {
  // Constants
  public static final double kMaxSpeed = 2.7; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  // State from robot logic

  // State coming from external or simulated devices
  public static final double kWheelBaseHalf = 0.33;
  public static final double kTrackWidthHalf = 0.273;

  private final Translation2d m_frontLeftLocation = new Translation2d(kWheelBaseHalf, kTrackWidthHalf); // x = .273, y =
                                                                                                        // .33
  private final Translation2d m_frontRightLocation = new Translation2d(kWheelBaseHalf, -kTrackWidthHalf);
  private final Translation2d m_backLeftLocation = new Translation2d(-kWheelBaseHalf, kTrackWidthHalf);
  private final Translation2d m_backRightLocation = new Translation2d(-kWheelBaseHalf, -kTrackWidthHalf);
  public final SwerveModule m_frontLeft = new SwerveModule(17, 16, "FL", !RobotBase.isSimulation(),
      !RobotBase.isSimulation());
  public final SwerveModule m_frontRight = new SwerveModule(3, 4, "FR", false, false);
  public final SwerveModule m_backLeft = new SwerveModule(19, 18, "BL", !RobotBase.isSimulation(),
      !RobotBase.isSimulation());
  public final SwerveModule m_backRight = new SwerveModule(1, 2, "BR", false, false);

  private final Gyro m_gyro = new Gyro();

  private ChassisSpeeds speeds = new ChassisSpeeds();
  Rotation2d angle = new Rotation2d();

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation,
      m_frontRightLocation,
      m_backLeftLocation,
      m_backRightLocation);
  // new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation,
  // m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      m_kinematics,
      angle,
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
      });

  /** Subsystem constructor. */
  public Drivetrain() {
    m_gyro.reset();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    speeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, angle)
        : new ChassisSpeeds(xSpeed, ySpeed, rot);

    var swerveModuleStates = m_kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);

    NtHelper.setDouble("/drive/FLdesiredangle", swerveModuleStates[0].angle.getDegrees());

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
    double[] desiredStates = {
        swerveModuleStates[0].angle.getRadians(),
        swerveModuleStates[0].speedMetersPerSecond,
        swerveModuleStates[1].angle.getRadians(),
        swerveModuleStates[1].speedMetersPerSecond,
        swerveModuleStates[2].angle.getRadians(),
        swerveModuleStates[2].speedMetersPerSecond,
        swerveModuleStates[3].angle.getRadians(),
        swerveModuleStates[3].speedMetersPerSecond,
    };

    NtHelper.setDoubleArray("/swerve/desiredStates", desiredStates);
    NtHelper.setDouble("/swerve/chassisSpeeds/rot", rot);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    NtHelper.setDouble("/pos/fl", m_frontLeft.getPosition().distanceMeters);
    NtHelper.setDouble("/pos/fr", m_frontRight.getPosition().distanceMeters);
    NtHelper.setDouble("/pos/bl", m_backLeft.getPosition().distanceMeters);
    NtHelper.setDouble("/pos/br", m_backRight.getPosition().distanceMeters);

    NtHelper.setDouble("/angle/fl", m_frontLeft.getPosition().angle.getDegrees());
    NtHelper.setDouble("/angle/fr", m_frontRight.getPosition().angle.getDegrees());
    NtHelper.setDouble("/angle/bl", m_backLeft.getPosition().angle.getDegrees());
    NtHelper.setDouble("/angle/br", m_backRight.getPosition().angle.getDegrees());
    m_odometry.update(
        angle,
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        });
  }

  /** Resets robot odometry. */
  public void resetOdometry(Pose2d pose) {
    m_frontLeft.resetPosition();
    m_frontRight.resetPosition();
    m_backLeft.resetPosition();
    m_backRight.resetPosition();
    m_odometry.resetPosition(
        angle, new SwerveModulePosition[] { m_frontLeft.getPosition(),
            m_frontRight.getPosition(), m_backLeft.getPosition(), m_backRight.getPosition() },
        pose);
  }

  public void resetAngle() {
    m_gyro.reset();
    angle = new Rotation2d();
  }

  /** Check the current robot pose. */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  private void realPeriodic() {
    angle = m_gyro.getRotation();

  }

  /** Update our simulation. This should be run every robot loop in simulation. */
  private void simulationPeriodic() {
    angle = angle.plus(new Rotation2d(speeds.omegaRadiansPerSecond * 0.02));
  }

  /** Update odometry - this should be run every robot loop. */
  public void periodic() {
    telemetry();
    NtHelper.setDouble("/robot/angle", angle.getDegrees());
    updateOdometry();
    m_frontLeft.update();
    m_frontRight.update();
    m_backLeft.update();
    m_backRight.update();
    if (RobotBase.isSimulation()) {
      simulationPeriodic();
    } else {
      realPeriodic();
    }
  }

  private void telemetry() {

    double[] measuredStates = {
        m_frontLeft.getState().angle.getRadians(),
        m_frontLeft.getState().speedMetersPerSecond,
        m_frontRight.getState().angle.getRadians(),
        m_frontRight.getState().speedMetersPerSecond,
        m_backLeft.getState().angle.getRadians(),
        m_backLeft.getState().speedMetersPerSecond,
        m_backRight.getState().angle.getRadians(),
        m_backRight.getState().speedMetersPerSecond,
    };

    NtHelper.setDoubleArray("/swerve/measuredStates", measuredStates);
    NtHelper.setDouble("/swerve/robotRotation", getPose().getRotation().getRadians());

  }
}