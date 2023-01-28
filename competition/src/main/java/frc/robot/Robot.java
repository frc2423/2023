// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.util.NtHelper;

import java.util.List;

public class Robot extends TimedRobot {
  private final XboxController m_controller = new XboxController(0);

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  private final Drivetrain m_drive = new Drivetrain();
  private final RamseteController m_ramsete = new RamseteController();
  private final Timer m_timer = new Timer();
  private final Auto m_auto = new Auto();


  @Override // is society
  public void robotInit() {
   
  }

  @Override
  public void robotPeriodic() {
    m_drive.periodic();
  }

  @Override
  public void autonomousInit() { //tell robot to go
    m_timer.reset();
    m_timer.start();
    m_auto.robotGo();
  }

  @Override
  public void autonomousPeriodic() { //check for completion, work toward goal
    getPeriod();
    double elapsed = m_timer.get();
    //ChassisSpeeds speeds = m_ramsete.calculate(m_drive.getPose());
    //m_drive.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, true);
    NtHelper.setDouble("/drive/distanceX", m_drive.getPose().getX());
    NtHelper.setDouble("/drive/distanceY", m_drive.getPose().getY());
    double xSpeed = 0;
    double ySpeed = 0;
    double rotSpeed = 0;
    if (Math.abs(m_drive.getPose().getX() - m_auto.getTarget().getX()) < 0.1){
      xSpeed = 0;
    }
    else{
      xSpeed = 1;
    }

    if (Math.abs(m_drive.getPose().getY() - m_auto.getTarget().getY()) < 0.1){
      ySpeed = 0;
    }
    else{
      ySpeed = 1;
    }

    if (Math.abs(m_drive.getPose().getRotation().getRadians() - m_auto.getTarget().getRotation().getRadians()) < 0.1){
      rotSpeed = 0;
    }
    else{
      rotSpeed = 1;
    }
    m_drive.drive(xSpeed, ySpeed, rotSpeed, true);
  }

  @Override
  public void teleopInit() {
    m_drive.resetAngle();
  }

  @Override
  public void teleopPeriodic() {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    double xSpeed = -m_xspeedLimiter.calculate(m_controller.getLeftY()) * Drivetrain.kMaxSpeed;
    if(Math.abs(m_controller.getLeftY()) < 0.1) {
      xSpeed = 0;
    }
    double ySpeed = m_yspeedLimiter.calculate(m_controller.getLeftX()) * Drivetrain.kMaxSpeed;
    if(Math.abs(m_controller.getLeftX()) < 0.1) {
      ySpeed = 0;
    }

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    double rot = m_rotLimiter.calculate(-m_controller.getRightX()) * Drivetrain.kMaxAngularSpeed;
    if(Math.abs(m_controller.getRightX()) < 0.2) {
      rot = 0;
    }
    ySpeed *= (isSimulation() ? -.5 : .5);
    m_drive.drive(xSpeed * .5, ySpeed, rot, isSimulation() ? true : true);
  }

  @Override
  public void testPeriodic() {
    double manualSpeed = NtHelper.getDouble("/test/speed", 0); // top speed is 3 
    double manualAngle = NtHelper.getDouble("/test/angle", 0);
    SwerveModuleState bloB = new SwerveModuleState(manualSpeed, Rotation2d.fromDegrees(manualAngle));
    m_drive.m_frontLeft.setDesiredState(bloB);
    m_drive.m_frontRight.setDesiredState(bloB);
    m_drive.m_backLeft.setDesiredState(bloB);
    m_drive.m_backRight.setDesiredState(bloB);
  }

  @Override
  public void testInit() {
    NtHelper.setDouble("/test/speed", 0);
    NtHelper.setDouble("/test/angle", 0);
  }
}
