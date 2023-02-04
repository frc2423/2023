// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.util.NtHelper;

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
  private Trajectory m_trajectory;
  private PWMSparkMax clawMotor;

  private static final int CLAW_MOTOR_PWM_PORT = 0;
  private static final double CLAW_OPEN_MOTOR_POWER = 0.5;
  private static final double CLAW_CLOSE_MOTOR_POWER = -CLAW_OPEN_MOTOR_POWER;

  @Override // is society
  public void robotInit() {
    m_trajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(2, 2, new Rotation2d()),
            List.of(),
            new Pose2d(6, 4, new Rotation2d()),
            new TrajectoryConfig(2, 2));
    clawMotor = new PWMSparkMax(CLAW_MOTOR_PWM_PORT);
  }

  @Override
  public void robotPeriodic() {
    m_drive.periodic();
  }

  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
    m_drive.resetOdometry(m_trajectory.getInitialPose());
  }

  @Override
  public void autonomousPeriodic() {
    getPeriod();
    double elapsed = m_timer.get();
    Trajectory.State reference = m_trajectory.sample(elapsed);
    ChassisSpeeds speeds = m_ramsete.calculate(m_drive.getPose(), reference);
    m_drive.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, true);
  }

  @Override
  public void teleopInit() {
    m_drive.resetAngle();
  }

  @Override
  public void teleopPeriodic() {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    double deadband = 0.2;
    double yControllerInput = MathUtil.applyDeadband(m_controller.getLeftY(), deadband);
    double xControllerInput = MathUtil.applyDeadband(m_controller.getLeftX(), deadband);

    double motorPower = 0.0;
    // if both bumpers are pressed, don't move the claw
    if (!(m_controller.getRightBumper() && m_controller.getLeftBumper())) {
      if (m_controller.getLeftBumper()) {
        motorPower = CLAW_OPEN_MOTOR_POWER;
      }
      if (m_controller.getRightBumper()) {
        motorPower = CLAW_CLOSE_MOTOR_POWER;
      }
    }
   
    clawMotor.set(motorPower);

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
  }

  @Override
  public void testInit() {
    NtHelper.setDouble("/test/speed", 0);
    NtHelper.setDouble("/test/angle", 0);
  }
}
