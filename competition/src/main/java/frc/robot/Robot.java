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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.util.NtHelper;

public class Robot extends TimedRobot {
  private final XboxController m_controller = new XboxController(0);
  private final XboxController m_controller_right = new XboxController(1);

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  private final Drivetrain m_drive = new Drivetrain();
  private final RamseteController m_ramsete = new RamseteController();
  private final Timer m_timer = new Timer();
  private Trajectory m_trajectory;
  // private PWMSparkMax clawMotor;
  // private NeoMotor telescopeMotor;
  // private NeoMotor shoulderMotor;
  private final Field2d field = new Field2d();

  private Arm arm;

  // private static final int CLAW_MOTOR_PWM_PORT = 0;
  // private static final double CLAW_OPEN_MOTOR_POWER = 0.5;
  // private static final double CLAW_CLOSE_MOTOR_POWER = -CLAW_OPEN_MOTOR_POWER;
  // private static final int TELESCOPE_MOTOR_CAN_BUS_PORT = 0;
  // private static final double TELESCOPE_EXTENSION_POWER = 0.5;
  // private static final double TELESCOPE_RETRACTION_POWER = -TELESCOPE_EXTENSION_POWER;
  // private static final double SHOULDER_FORWARD_POWER = 0.5;
  // private static final double SHOULDER_BACKWARD_POWER = -SHOULDER_FORWARD_POWER;
  // public static final double DISTANCE = 0;

  @Override // is society
  public void robotInit() {
    m_trajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(2, 2, new Rotation2d()),
            List.of(),
            new Pose2d(6, 4, new Rotation2d()),
            new TrajectoryConfig(2, 2));
    // clawMotor = new PWMSparkMax(CLAW_MOTOR_PWM_PORT);
    // telescopeMotor = new NeoMotor(TELESCOPE_MOTOR_CAN_BUS_PORT,false);
    // telescopeMotor.setPercent(kDefaultPeriod);
    arm = new Arm();
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
    NtHelper.setDouble("/robot/shoulder/set_angle", 0);
    arm.resetTelescopeEncoder();
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

   if (m_controller.getLeftBumper()) {                                                                                                //hello adrian
      arm.shoulderForward();
   } 
   else if (m_controller.getRightBumper()) {
      arm.shoulderBack();
   } else if (m_controller.getBButton()) {
      double benny = NtHelper.getDouble("/robot/shoulder/set_angle", 0);
      arm.shoulderSetpoint(new Rotation2d(Units.degreesToRadians(benny)));
    } else if (m_controller.getXButton()) {
      arm.shoulderSetpoint(new Rotation2d(0));
    }
   else {                                                                                                                              // funny seeing you here
      arm.shoulderStop();
   }

   arm.getShoulderAngle();
   arm.getTelescopePosition();
   
    if (m_controller.getAButton()) {
      arm.retract();
   } else if (m_controller.getYButton()) { //double check this
      arm.extend();  
   } else {
      arm.stopTelescopeMotor();
   }
   if (m_controller_right.getYButton()) {
    arm.intakeBelt();
   } else if (m_controller_right.getAButton()) {
    arm.outtakeBelt();
   } else if (m_controller_right.getXButton()) {
    arm.beltStop();
   } 

   if (m_controller_right.getBButton()){
    arm.shoulderSetpoint(new Rotation2d(0));
   } else if (m_controller_right.getLeftBumper()) {
    arm.shoulderSetpoint(new Rotation2d(Units.degreesToRadians(110)));
   } else if (m_controller_right.getRightBumper()) {
    arm.shoulderSetpoint(new Rotation2d(Units.degreesToRadians(65)));
   }


  }

  @Override
  public void testPeriodic() {
    double manualSpeed = NtHelper.getDouble("/test/speed", 0); // top speed is 3 
    double manualAngle = NtHelper.getDouble("/test/angle", 0);
    SwerveModuleState bloB = new SwerveModuleState(manualSpeed, Rotation2d.fromDegrees(manualAngle));
    // m_drive.m_frontLeft.setDesiredState(bloB);
    // m_drive.m_frontRight.setDesiredState(bloB);
    // m_drive.m_backLeft.setDesiredState(bloB);
    // m_drive.m_backRight.setDesiredState(bloB);        
    if (m_controller_right.getStartButton()){
      arm.telescope_override();
   }
   else if (m_controller_right.getBackButton()){
    arm.resetTelescopeEncoder();
   }             
   else {
    arm.stopTelescopeMotor();
   }                                                                      //:P
  }

  @Override
  public void testInit() {
    NtHelper.setDouble("/test/speed", 0);
    NtHelper.setDouble("/test/angle", 0);
    arm.reset_shoulder();
  }
}
