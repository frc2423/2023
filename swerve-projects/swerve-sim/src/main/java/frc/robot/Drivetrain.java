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
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain {
  // Constants
  public static final double kMaxSpeed = 2.7; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  // State from robot logic

  // State coming from external or simulated devices

  private final Translation2d m_frontLeftLocation = new Translation2d(0.273, 0.33);
  private final Translation2d m_frontRightLocation = new Translation2d(0.273, -0.33);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.273, 0.33);
  private final Translation2d m_backRightLocation = new Translation2d(-0.273, -0.33);

  private final SwerveModule m_frontLeft = new SwerveModule(17,16);
  private final SwerveModule m_frontRight = new SwerveModule(3,4);
  private final SwerveModule m_backLeft = new SwerveModule(19,18);
  private final SwerveModule m_backRight = new SwerveModule(1, 2);

  private final Gyro m_gyro = new Gyro();

  private ChassisSpeeds speeds = new ChassisSpeeds();
  Rotation2d angle = new Rotation2d();

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          angle,
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });

  // Simulation classes help us simulate our robot
  // private final AnalogGyroSim m_gyroSim = new AnalogGyroSim();
  private final Field2d m_fieldSim = new Field2d();

  /** Subsystem constructor. */
  public Drivetrain() {
    m_gyro.reset();
    SmartDashboard.putData("Field", m_fieldSim);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    speeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, angle) 
    : new ChassisSpeeds(xSpeed, ySpeed, rot);
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);

    
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);

    NtHelper.setDouble("/robot/blDesiredAngle", swerveModuleStates[2].angle.getDegrees());
    NtHelper.setDouble("/robot/blDesiredSpeed", swerveModuleStates[2].speedMetersPerSecond);
    NtHelper.setDouble("/robot/blCurentAngle", m_backLeft.getState().angle.getDegrees());
    NtHelper.setDouble("/robot/blCurentSpeed", m_backLeft.getState().speedMetersPerSecond);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
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
        angle, new SwerveModulePosition[]{m_frontLeft.getPosition(), 
          m_frontRight.getPosition(), m_backLeft.getPosition(), m_backRight.getPosition()}, pose);
  }

  /** Check the current robot pose. */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  private void realPeriodic() {
    m_fieldSim.setRobotPose(m_odometry.getPoseMeters());
    angle = m_gyro.getRotation();
  }

  /** Update our simulation. This should be run every robot loop in simulation. */
  private void simulationPeriodic() {
    m_fieldSim.setRobotPose(m_odometry.getPoseMeters());
    // To update our simulation, we set motor voltage inputs, update the
    // simulation, and write the simulated positions and velocities to our
    // simulated encoder and gyro. We negate the right side so that positive
    // voltages make the right side move forward.
    angle = angle.plus(new Rotation2d(-speeds.omegaRadiansPerSecond * /*dtSeconds*/0.02));
    //m_gyroSim.setAngle(angle);
  }

  /** Update odometry - this should be run every robot loop. */
  public void periodic() {
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
}
