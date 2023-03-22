// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.Auto;
import frc.robot.util.Camera;
import frc.robot.util.NtHelper;
import frc.robot.util.PhotonRunnable;
import frc.robot.util.stateMachine.StateMachine; 

public class Robot extends TimedRobot {
  private final XboxController m_controller = new XboxController(0);
  private final XboxController m_controller_right = new XboxController(1);
  private final PhotonRunnable photonEstimator = new PhotonRunnable();
  private final Notifier photonNotifier = new Notifier(photonEstimator);

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  public static Drivetrain m_drive = new Drivetrain();
  public static Trajectories trajectories = new Trajectories();
  public static final Field2d field = new Field2d();
  public static final Camera m_camera= new Camera("driverCamera");
  private AutoAlign autoAlign = new AutoAlign();

  private StateMachine autoScoreCube = new AutoScoreCube();

  public static Arm arm = new Arm();

  private Auto auto = new Auto();

  @Override // is society
  public void robotInit() {
    photonNotifier.setName("PhotonRunnable");
    photonNotifier.startPeriodic(0.02);
    DataLogManager.start();
    NtHelper.setBoolean("/dashboard/arm/isCubes", true);
    NtHelper.setBoolean("/robot/arm/telescopeoveride", true); 
    m_drive.setBrake(false);
    CameraServer.startAutomaticCapture();
    NtHelper.setDouble("/dashboard/armSetpoint/buttonselected", 5);

    NtHelper.listen("/dashboard/armSetpoint/buttonselected", (entry) -> {
      updateArmSetpoint();

    });

    NtHelper.listen("/dashboard/arm/isCubes", (entry) -> {
      updateArmSetpoint();

    });

    String DASHBOARD_DISABLED_ARM_KEY = "/dashboard/disabled/arm";
    NtHelper.listen(DASHBOARD_DISABLED_ARM_KEY, (entry) ->{
      boolean armDisabled = NtHelper.getBoolean(DASHBOARD_DISABLED_ARM_KEY, false);
      arm.setEnabled(armDisabled);

    });
    NtHelper.listen("/robot/arm/telescopeoveride", (entry)->{
      boolean safeMode = NtHelper.getBoolean("/robot/arm/telescopeoveride", true);
      if (safeMode && isTeleop()) {
        arm.resetTelescopeEncoder();
      }
    });
    SmartDashboard.putData("Field", field);
  }

  @Override
  public void robotPeriodic() {
    telemtry();
    if (!isTeleop()) {
      arm.isSafeMode(!isTest());
    }
    m_drive.periodic();
    arm.periodic();
    field.setRobotPose(m_drive.getPose());
  }



  public void updateArmSetpoint() {
      var position = NtHelper.getDouble("/dashboard/armSetpoint/buttonselected", 5);

      boolean isCubes = NtHelper.getBoolean("/dashboard/arm/isCubes", false);
      var midTeleSetPoint = (isCubes ? 0 : 17);
      var midShoulderSetPoint = 52;
      var highTeleSetPoint = (isCubes ? 20 : 30);
      

      
      if (position == 5) {
        arm.setShoulderSetpoint(new Rotation2d(Units.degreesToRadians(5)));
        arm.telescopeToSetpoint(0);
      } else if (position == 2) {
        arm.setShoulderSetpoint(new Rotation2d(Units.degreesToRadians(midShoulderSetPoint)));
        arm.telescopeToSetpoint(midTeleSetPoint);
      } else if (position == 1) {
        arm.setShoulderSetpoint(new Rotation2d(Units.degreesToRadians(122))); // 122
        arm.telescopeToSetpoint(0);
      } else if (position == 9) {
        arm.setShoulderSetpoint(new Rotation2d(Units.degreesToRadians(-122)));
        arm.telescopeToSetpoint(0);
      } else if (position == 8) {
        arm.setShoulderSetpoint(new Rotation2d(Units.degreesToRadians(-midShoulderSetPoint)));
        arm.telescopeToSetpoint(midTeleSetPoint);
      } else if (position == 3) {
        arm.setShoulderSetpoint(new Rotation2d(Units.degreesToRadians(57)));
        arm.telescopeToSetpoint(highTeleSetPoint);
      } else if (position == 7) {
        arm.setShoulderSetpoint(new Rotation2d(Units.degreesToRadians(-57)));
        arm.telescopeToSetpoint(highTeleSetPoint);
      

      }
    
  }

  @Override
  public void autonomousInit() {
    m_drive.resetAngle();
    arm.resetTelescopeEncoder();
    m_drive.setBrake(false);
    auto.restart();
  }

  @Override
  public void autonomousPeriodic() {
    auto.run();
  }

  @Override
  public void teleopInit() {
    m_drive.setBrake(false);
    NtHelper.setDouble("/robot/shoulder/set_angle", 0);
    NtHelper.setString("/robot/arm/setsolenoid", "off");
    
  }

  @Override
  public void teleopPeriodic() {
    final double kMaxSpeed = 4;

    Robot.m_drive.addVisionMeasurement(photonEstimator.grabLatestEstimatedPose());
    
    if (m_controller.getStartButtonReleased()) {
      Robot.m_drive.setBrake(false);
    }
    
    if (m_controller.getStartButtonPressed()) {
      Robot.m_drive.setBrake(true);
    }
    
    if (m_controller.getBackButtonPressed()) {
      Robot.m_drive.setBrake(true);
      autoScoreCube.setState(autoScoreCube.getDefaultState());
    }

    if (m_controller.getBackButtonReleased()) {
      Robot.m_drive.setBrake(false);
    }

    if (m_controller.getStartButton()) {
      // autoAlign.autoRotate();

    
     
      
      SwerveModuleState brflSTATE = new SwerveModuleState(0,
                Rotation2d.fromDegrees(0));
      SwerveModuleState frblSTATE = new SwerveModuleState(0,
                Rotation2d.fromDegrees(90));

        Robot.m_drive.m_frontLeft.setDesiredState(brflSTATE);
        Robot.m_drive.m_frontRight.setDesiredState(frblSTATE);
        Robot.m_drive.m_backLeft.setDesiredState(frblSTATE);
        Robot.m_drive.m_backRight.setDesiredState(brflSTATE);
        
          
    }else if (m_controller.getBackButton()){
      autoScoreCube.run();
      // Robot.m_drive.addBestVisionMeasurement(m_camera);
    } else {
      boolean isSlowMode = m_controller.getLeftTriggerAxis() > 0.2;
      double maxSpeed = (isSlowMode ? 1.5 : kMaxSpeed);
      double maxRotation = (isSlowMode ? Math.PI : Drivetrain.kMaxAngularSpeed);

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

      if (m_controller.getYButton() || m_controller_right.getYButton()) {
        arm.intakeBelt();
      } else if (m_controller_right.getAButton()) {
        arm.closeGripper();
      } else {
        arm.beltStop();
      }
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

  

    if (m_controller.getLeftBumperReleased()) { // TODO: revisit this
      arm.shoulderForward();
    } else if (m_controller.getRightBumperReleased()) {
      arm.shoulderBack();
    }

    if (m_controller_right.getXButton()) {
      arm.openGripper();
    } else if (m_controller_right.getBButton() || m_controller.getBButton()) {
      arm.outtakeBelt();
    }

    
     arm.isSafeMode(NtHelper.getBoolean("/robot/arm/telescopeoveride", true)); 
    

  }

  @Override
  public void testPeriodic() {
    if (m_controller.getStartButton()) {
      arm.resetShoulder();
    }
    NtHelper.setDouble("/test/shoulderPosition", arm.getShoulderEncoderPosition());

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
    } else if (m_controller.getAButton()) { // double check this
      arm.retract();
    } else {
      arm.stopTelescopeMotor();
    }

    
  }

  @Override
  public void testInit() {
    m_drive.setBrake(false);
    m_drive.resetAngle();
    arm.resetTelescopeEncoder();
    NtHelper.setDouble("/test/speed", 0);
    NtHelper.setDouble("/test/angle", 0);
  }

  public void telemtry() {
    NtHelper.setDouble("/dashboard/arm/angleMeasured", -arm.getShoulderAngle().getDegrees() + 90);
    NtHelper.setDouble("/dashboard/arm/telscopeLenMeasured", arm.getTelescopePosition() * 5);
    NtHelper.setDouble("/dashboard/arm/angleSetpoint", -arm.getShoulderSetpoint().getDegrees() + 90);
    NtHelper.setDouble("/dashboard/arm/telescopeLenSetpoint", arm.getTelescopeSetpoint() * 5);
    NtHelper.setDouble("/dashboard/robot/roll", m_drive.m_gyro.getRoll());
    NtHelper.setDouble("/dashboard/robot/pitch", m_drive.m_gyro.getPitch());
    NtHelper.setBoolean("/SmartDashboard/Field/flip", Alliance.Red.equals(DriverStation.getAlliance()));
    NtHelper.setDouble("/dashboard/robot/velocity", Robot.m_drive.m_backLeft.getState().speedMetersPerSecond);
  }

}
