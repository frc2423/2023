// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.NtHelper;
import frc.robot.auto.Auto;
import edu.wpi.first.cameraserver.CameraServer;

public class Robot extends TimedRobot {
  private final XboxController m_controller = new XboxController(0);
  private final XboxController m_controller_right = new XboxController(1);

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  public static Drivetrain m_drive = new Drivetrain();
  private final Timer m_timer = new Timer();
  private Trajectory m_trajectory;
  public static final Field2d field = new Field2d();
  public static Trajectories trajectories = new Trajectories();

  private AutoAlign autoAlign = new AutoAlign();

  public static Arm arm = new Arm();

  private Auto auto = new Auto();

  @Override // is society
  public void robotInit() {
    m_drive.setBrake(false);
    CameraServer.startAutomaticCapture();
    m_trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(2, 2, new Rotation2d()),
        List.of(),
        new Pose2d(6, 4, new Rotation2d()),
        new TrajectoryConfig(2, 2));
    NtHelper.setDouble("/dashboard/armSetpoint/buttonselected", 5);

    NtHelper.listen("/dashboard/armSetpoint/buttonselected", (entry) -> {
      var position = NtHelper.getDouble("/dashboard/armSetpoint/buttonselected", 5);
      if (position == 5) {
        arm.setShoulderSetpoint(new Rotation2d(Units.degreesToRadians(5)));
        arm.telescopeToSetpoint(0);
      } else if (position == 2) {
        arm.setShoulderSetpoint(new Rotation2d(Units.degreesToRadians(57)));
        arm.telescopeToSetpoint(20);
      } else if (position == 1) {
        arm.setShoulderSetpoint(new Rotation2d(Units.degreesToRadians(115))); // 122
        arm.telescopeToSetpoint(0);
      } else if (position == 9) {
        arm.setShoulderSetpoint(new Rotation2d(Units.degreesToRadians(-115)));
        arm.telescopeToSetpoint(0);
      } else if (position == 8) {
        arm.setShoulderSetpoint(new Rotation2d(Units.degreesToRadians(-57)));
        arm.telescopeToSetpoint(20);
      } else if (position == 3) {
        arm.setShoulderSetpoint(new Rotation2d(Units.degreesToRadians(57)));
        arm.telescopeToSetpoint(43);
      } else if (position == 7) {
        arm.setShoulderSetpoint(new Rotation2d(Units.degreesToRadians(-57)));
        arm.telescopeToSetpoint(43);

      }
    });
    
    String DASHBOARD_DISABLED_ARM_KEY = "/dashboard/disabled/arm";
    NtHelper.listen(DASHBOARD_DISABLED_ARM_KEY, (entry) ->{
      boolean armDisabled = NtHelper.getBoolean(DASHBOARD_DISABLED_ARM_KEY, false);
      arm.setEnabled(armDisabled);

    });
    SmartDashboard.putData("Field", field);
  }

  @Override
  public void robotPeriodic() {
    telemtry();
    arm.isSafeMode(!isTest());
    m_drive.periodic();
    arm.periodic();
    field.setRobotPose(m_drive.getPose());
  }

  @Override
  public void autonomousInit() {
    m_drive.setBrake(false);
    m_timer.reset();
    m_timer.start();
    m_drive.resetOdometry(m_trajectory.getInitialPose());
    auto.restart();
  }

  @Override
  public void autonomousPeriodic() {
    auto.run();
  }

  @Override
  public void teleopInit() {
    m_drive.setBrake(false);
    m_drive.resetAngle();
    NtHelper.setDouble("/robot/shoulder/set_angle", 0);
    arm.resetTelescopeEncoder();
    NtHelper.setString("/robot/arm/setsolenoid", "off");
  }

  @Override
  public void teleopPeriodic() {

    if (m_controller.getStartButton()) {
      autoAlign.autoRotate();
    } else {
      boolean isSlowMode = m_controller.getLeftTriggerAxis() > 0.2;
      double maxSpeed = Drivetrain.kMaxSpeed * (isSlowMode ? .55 : .75);
      double maxRotation = Drivetrain.kMaxAngularSpeed * (isSlowMode ? .55 : 1);

      double deadband = 0.2;

      // Get the y speed. We are inverting this because Xbox controllers return
      // negative values when we push forward.
      double xInput = -MathUtil.applyDeadband(m_controller.getLeftY(), deadband);
      double yInput = -MathUtil.applyDeadband(m_controller.getLeftX(), deadband);
      // Get the rate of angular rotation. We are inverting this because we want a
      // positive value when we pull to the left (remember, CCW is positive in
      // mathematics). Xbox controllers return positive values when you pull to
      // the right by default.
      double rotInput = -MathUtil.applyDeadband(m_controller.getRightX(), deadband);
      
      double xSpeed = m_xspeedLimiter.calculate(xInput) * maxSpeed;
      double ySpeed = m_yspeedLimiter.calculate(yInput) * maxSpeed;
      double rot = m_rotLimiter.calculate(rotInput) * maxRotation;
      
      m_drive.drive(xSpeed, ySpeed, rot, true);
    }

int buttonindex = -1;

    boolean shiftUp = m_controller_right.getLeftTriggerAxis() > 0.2;
    boolean shiftDown = m_controller_right.getRightTriggerAxis() > 0.2;
    
    switch (m_controller_right.getPOV()) {
      case 0:
      buttonindex = 5;
      break;
      case 90:
      if (shiftUp) {
        buttonindex = 3;
      } else if (shiftDown) {
        buttonindex = 1;
      } else {
        buttonindex = 2;
      }
      break;
      case 270:
      if (shiftUp) {
        buttonindex = 7;
      } else if (shiftDown) {
        buttonindex = 9;
      } else {
        buttonindex = 8;
      }
      break;
    }
      
      if (buttonindex != -1) {
        NtHelper.setDouble("/dashboard/armSetpoint/buttonselected", buttonindex);
      }
      
    if (m_controller.getXButton()) {
      arm.extend();
    } else if (m_controller.getAButton()) { // double check this
      arm.retract();
    }

    if (m_controller.getXButtonReleased()) {
      arm.stopTelescopeMotor();
    }

    if (m_controller.getBButtonReleased()) {
      arm.stopTelescopeMotor();
    }

    if (m_controller.getYButton() || m_controller_right.getYButton()) {
      arm.intakeBelt();
    } else if (m_controller_right.getAButton()) {
      arm.closeGripper();
    } else {
      arm.beltStop();
    }

    if (m_controller_right.getLeftBumperReleased()) { // TODO: revisit this
      arm.shoulderForward();
    } else if (m_controller_right.getRightBumperReleased()) {
      arm.shoulderBack();
    }

    if (m_controller_right.getXButton()) {
      arm.openGripper();
    } else if (m_controller_right.getBButton() || m_controller.getBButton()) {
      arm.outtakeBelt();
    }

  }

  @Override
  public void testPeriodic() {
    if (m_controller.getStartButton()) {
      arm.resetShoulder();
    }
    NtHelper.setDouble("/test/shoulderPosition", arm.getShoulderEncoderPosition());
    double radians = Units.degreesToRadians(5);

    double manualSpeed = NtHelper.getDouble("/test/speed", 0); // top speed is 3
    double manualAngle = NtHelper.getDouble("/test/angle", 0);
    SwerveModuleState bloB = new SwerveModuleState(manualSpeed,
        Rotation2d.fromDegrees(manualAngle));

    double[] desiredStates = {
        bloB.angle.getRadians(),
        bloB.speedMetersPerSecond,
        bloB.angle.getRadians(),
        bloB.speedMetersPerSecond,
        bloB.angle.getRadians(),
        bloB.speedMetersPerSecond,
        bloB.angle.getRadians(),
        bloB.speedMetersPerSecond,
    };

    NtHelper.setDoubleArray("/swerve/desiredStates", desiredStates);

    m_drive.m_frontLeft.setDesiredState(bloB);
    m_drive.m_frontRight.setDesiredState(bloB);
    m_drive.m_backLeft.setDesiredState(bloB);
    m_drive.m_backRight.setDesiredState(bloB);

    if (m_controller.getXButton()) {
      arm.extend();
    } else if (m_controller.getBButton()) { // double check this
      arm.retract();
    } else {
      arm.stopTelescopeMotor();
    }

    
  }

  @Override
  public void testInit() {
    m_drive.setBrake(false);
    NtHelper.setDouble("/test/speed", 0);
    NtHelper.setDouble("/test/angle", 0);
  }

  public void telemtry() {
    NtHelper.setDouble("/dashboard/arm/angleMeasured", -arm.getShoulderAngle().getDegrees() + 90);
    NtHelper.setDouble("/dashboard/arm/telscopeLenMeasured", arm.getTelescopePosition());
    NtHelper.setDouble("/dashboard/arm/angleSetpoint", -arm.getShoulderSetpoint().getDegrees() + 90);
    NtHelper.setDouble("/dashboard/arm/telescopeLenSetpoint", arm.getTelescopeSetpoint());
    NtHelper.setDouble("/dashboard/robot/roll", m_drive.m_gyro.getRoll());
    NtHelper.setDouble("/dashboard/robot/pitch", m_drive.m_gyro.getPitch());
  }

}
