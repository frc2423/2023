// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.NtHelper;
import edu.wpi.first.math.MathUtil;

public class Robot extends TimedRobot {
  private final XboxController m_controller = new XboxController(0);

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  private final Drivetrain m_drive = new Drivetrain();
  private final Timer m_timer = new Timer();
  private final Auto m_auto = new Auto(m_drive);

  private final Field2d field = new Field2d();



  @Override // is society
  public void robotInit() {
    SmartDashboard.putData("Field", field);

  }

  @Override
  public void robotPeriodic() {
    m_drive.periodic();
    field.setRobotPose(m_drive.getPose());
  }

  @Override
  public void autonomousInit() { //tell robot to go
    m_drive.resetAngle();
    m_drive.resetOdometry(new Pose2d());
    m_timer.reset();
    m_timer.start();
    m_auto.robotGo();
    m_auto.update_current_path();
    field.getObject("trajectory").setTrajectory(m_auto.getTrajectory());
  }

  @Override
  public void autonomousPeriodic() { //check for completion, work toward goal
    getPeriod();
    m_auto.follow_current_path();
  }

  @Override
  public void teleopInit() {
    m_drive.resetAngle();
    m_drive.resetOdometry(new Pose2d());
  }

  @Override
  public void teleopPeriodic() {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    double deadband = 0.2;
    double yControllerInput = MathUtil.applyDeadband(m_controller.getLeftY(), deadband);
    double xControllerInput = MathUtil.applyDeadband(m_controller.getLeftX(), deadband);

    double xSpeed = -m_xspeedLimiter.calculate(yControllerInput) * Drivetrain.kMaxSpeed;
  
    double ySpeed = m_yspeedLimiter.calculate(xControllerInput) * Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    double rotInput = MathUtil.applyDeadband(-m_controller.getRightX(), deadband);
    double rot = m_rotLimiter.calculate(rotInput) * Drivetrain.kMaxAngularSpeed;

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
    // NtHelper.setDouble("/drive/frontLeft/distance", Units.metersToFeet(m_drive.m_backRight.getDistance()));
    // NtHelper.setDouble("/drive/frontLeft/distance", m_drive.m_backRight.getDistance());
    
  }
  

  @Override
  public void testInit() {
    m_drive.resetOdometry(new Pose2d());
    NtHelper.setDouble("/test/speed", 0);
    NtHelper.setDouble("/test/angle", 0);
  }
}
