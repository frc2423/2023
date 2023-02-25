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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Camera;
import frc.robot.util.LinearScale;
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

  private final Camera camera = new Camera(null);

  private final LinearScale objectalignment = new LinearScale(.1, 1, 5, 15);
  private final LinearScale robotRotate = new LinearScale(0, Math.PI, Rotation2d.fromDegrees(10).getRadians(),
      Rotation2d.fromDegrees(100).getRadians());
  public static Arm arm = new Arm();

  private Auto auto = new Auto();

  @Override // is society
  public void robotInit() {
    CameraServer.startAutomaticCapture();
    m_trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(2, 2, new Rotation2d()),
        List.of(),
        new Pose2d(6, 4, new Rotation2d()),
        new TrajectoryConfig(2, 2));
    NtHelper.setString("/arm/value", "Up");
    NtHelper.setString("/robot/score/AutoPos", "low");
    String[] options = { "Front Floor", "Front Score", "Up", "Back Score", "Back Floor" };
    NtHelper.setStringArray("/arm/options", options);

    NtHelper.listen("/arm/value", (entry) -> {
      var position = NtHelper.getString("/arm/value", "Up");
      if (position.equals("Up")) {
        arm.setShoulderSetpoint(new Rotation2d(Units.degreesToRadians(5)));
        arm.telescopeToSetpoint(0);
      } else if (position.equals("Front Score")) {
        frontScoreMid();
      } else if (position.equals("Front Floor")) {
        frontFloor();
      } else if (position.equals("Back Floor")) {
        arm.setShoulderSetpoint(new Rotation2d(Units.degreesToRadians(-110)));
        arm.telescopeToSetpoint(10);
      } else if (position.equals("Back Score")) {
        arm.setShoulderSetpoint(new Rotation2d(Units.degreesToRadians(-57)));
        arm.telescopeToSetpoint(51);

      }
    });
    SmartDashboard.putData("Field", field);
  }

  @Override
  public void robotPeriodic() {
    m_drive.periodic();
    arm.periodic();
    field.setRobotPose(m_drive.getPose());
  }

  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
    m_drive.resetOdometry(m_trajectory.getInitialPose());
    auto.restart();
  }

  @Override
  public void autonomousPeriodic() {
    auto.run();
  }

  public void frontFloor() {
    arm.setShoulderSetpoint(new Rotation2d(Units.degreesToRadians(110)));
    arm.telescopeToSetpoint(10);
  }

  public void frontScoreMid() {
    arm.setShoulderSetpoint(new Rotation2d(Units.degreesToRadians(57))); // 57 but 59 good
    arm.telescopeToSetpoint(51);
  }

  public void autoScore() {
    m_drive.drive(0, 0, 0, false);
    String AutoPos = NtHelper.getString("/robot/score/AutoPos", "low");
    if (AutoPos.equals("low")) {
      frontFloor();
    }else if (AutoPos == "mid"){
      frontScoreMid();
    }
    else if (AutoPos == "high") {
      //make a function for high score
    }
    else {
    }
    }
  

  public void autoAlign() {
    if (camera.seesTarget()) {
      var tapeX = camera.getTapeX();
      if (!objectalignment.isDone(tapeX)) {
        var speed = objectalignment.calculate(tapeX);
        m_drive.drive(0, -speed, 0, false);

        NtHelper.setDouble("/autoAlign/speed", speed);
        NtHelper.setDouble("/autoAlign/tapeX", tapeX);
      } else {
        autoScore();
      }
    } else {
      m_drive.drive(0, 0, 0, false);
    }
  }

  public void autoRotate() {
    var angleError = MathUtil.angleModulus(m_drive.getAngle().getRadians());
    NtHelper.setDouble("/robot/score/angleEroor", angleError);
    if (!robotRotate.isDone(angleError)) {
      var rotationSpeed = robotRotate.calculate(angleError);
      m_drive.drive(0, 0, rotationSpeed / 4, false);
    } else {
      System.out.println(1);
      autoAlign();
    }
  }

  @Override
  public void teleopInit() {
    m_drive.resetAngle();
    NtHelper.setDouble("/robot/shoulder/set_angle", 0);
    arm.resetTelescopeEncoder();
  }

  @Override
  public void teleopPeriodic() {

    if (m_controller.getStartButton()) {
      autoRotate();
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

    if (m_controller.getXButton()) {
      arm.extend();
    } else if (m_controller.getBButton()) { // double check this
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
    } else if (m_controller.getAButton() || m_controller_right.getAButton()) {
      arm.outtakeBelt();
    } else {
      arm.beltStop();
    }

    if (m_controller.getLeftBumperReleased()) { // TODO: revisit this
      arm.shoulderForward();
    } else if (m_controller.getRightBumperReleased()) {
      arm.shoulderBack();
    }
  }

  @Override
  public void testPeriodic() {
    if (m_controller.getStartButton()) {
      arm.resetShoulder();
    }
    NtHelper.setDouble("/test/shoulderPosition", arm.getShoulderEncoderPosition());
    double radians = Units.degreesToRadians(5);
    // if (m_controller.getYButton()){
    // arm.setShoulderVelocity(radians);
    // } else if (m_controller.getAButton()) {
    // arm.setShoulderVelocity(-radians);
    // } else {
    // arm.setShoulderVelocity(0);
    // }
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
    // if (m_controller_right.getBackButton()) {
    // arm.resetTelescopeEncoder();
    // } else {
    // arm.stopTelescopeMotor();
    // } // :P
  }

  @Override
  public void testInit() {
    NtHelper.setDouble("/test/speed", 0);
    NtHelper.setDouble("/test/angle", 0);
  }
}
