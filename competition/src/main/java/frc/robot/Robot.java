// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Led.KwarqsLed;
import frc.robot.Rumble.Position;
import frc.robot.auto.Auto;
import frc.robot.auto.HumanPlayerStationDetection;
import frc.robot.constants.SetPoints;
import frc.robot.util.NtHelper;
import frc.robot.util.PhotonRunnable;
import frc.robot.util.stateMachine.StateMachine;

public class Robot extends TimedRobot {
  private final XboxController m_controller = new XboxController(0);
  private final XboxController m_controller_right = new XboxController(1);
  public final static PhotonRunnable photonEstimator = new PhotonRunnable();
  private final Notifier photonNotifier = new Notifier(photonEstimator);
  private final KwarqsLed ledBrain = new KwarqsLed();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  public static Drivetrain m_drive = new Drivetrain();
  public static Trajectories trajectories = new Trajectories();
  public static final Field2d field = new Field2d();
  private AutoScoreCube autoScoreCube = new AutoScoreCube();
  private StateMachine autoHuman = new AutoHuman();

  public static Arm arm = new Arm();

  private Auto auto = new Auto();

  boolean prevAutoHuman = false;
  boolean isAutoHuman = false;

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
    NtHelper.setString("/robot/dashboard/led", "green");
    NtHelper.setBoolean("/dashboard/gyroIsResetting", false);
    NtHelper.setDouble("/test/LeftRumble", 0);
    NtHelper.setDouble("/test/BothRumble", 0);
    NtHelper.setDouble("/test/RightRumble", 0);

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
    setLED();
    NtHelper.listen("/dashboard/arm/isCubes", (entry)->{
        setLED();
        
    });

    


    SmartDashboard.putData("Field", field);
  }

  public void setLED() {
    if(isDisabled()) {
      ledBrain.disable();
    }
    else if (NtHelper.getBoolean("/dashboard/arm/isCubes", true)) {
      ledBrain.setPurple();
    } else {
      ledBrain.setYellow();
    }
  }

  @Override
  public void robotPeriodic() {
    
    telemtry();
    if (isDisabled()) {
      arm.isSafeMode(!isTest());
      m_controller.setRumble(RumbleType.kBothRumble, 0);
    }
    m_drive.periodic();
    arm.periodic();
    field.setRobotPose(m_drive.getPose());
    setLED();
    ledBrain.run();

    try {
      NtHelper.setString("/dashboard/getScoringTagLabel", autoScoreCube.getScoringTagLabel());
    } catch(Exception exception) {
      System.err.println("Error getting scoring tag");
      exception.printStackTrace();
    }
  }


  @Override
  public void disabledInit() {
    
  }
  

  public void updateArmSetpoint() {
      var position = NtHelper.getDouble("/dashboard/armSetpoint/buttonselected", 5);

      boolean isCubes = NtHelper.getBoolean("/dashboard/arm/isCubes", false);
      var midTeleSetPoint = (isCubes ? SetPoints.TELESCOPE_MID_CUBE_LENGTH : SetPoints.TELESCOPE_MID_CONE_LENGTH);
      var midFrontShoulderSetPoint = (isCubes ? SetPoints.SHOULDER_FRONT_MID_CUBE_ANGLE : SetPoints.SHOULDER_FRONT_MID_CONE_ANGLE);
      var midBackShoulderSetPoint = (isCubes ? SetPoints.SHOULDER_BACK_MID_CUBE_ANGLE : SetPoints.SHOULDER_BACK_MID_CONE_ANGLE);
      var highTeleSetPoint = (isCubes ? SetPoints.TELESCOPE_HIGH_CUBE_LENGTH : SetPoints.TELESCOPE_HIGH_CONE_LENGTH);
      var highFrontShoulderSetPoint = (isCubes ? SetPoints.SHOULDER_FRONT_HIGH_CUBE_ANGLE : SetPoints.SHOULDER_FRONT_HIGH_CONE_ANGLE);
      var highBackShoulderSetPoint = (isCubes ? SetPoints.SHOULDER_BACK_HIGH_CUBE_ANGLE : SetPoints.SHOULDER_BACK_HIGH_CONE_ANGLE);
      if (!isCubes && (position == SetPoints.ARM.BACK_MID) || (position == SetPoints.ARM.FRONT_MID)) {
        arm.setOutakeSpeed(-0.3);
      }
      else {
        arm.setOutakeSpeed(-1);
      }
      
      if (position == SetPoints.ARM.UP) { //5
        arm.setShoulderSetpoint(SetPoints.SHOULDER_UP_ANGLE);
        arm.telescopeToSetpoint(0);
      } else if (position == SetPoints.ARM.FRONT_MID) { //2
        arm.setShoulderSetpoint(midFrontShoulderSetPoint);
        arm.telescopeToSetpoint(midTeleSetPoint);
      } else if (position == SetPoints.ARM.FRONT_FLOOR) { //1
        arm.setShoulderSetpoint(SetPoints.SHOULDER_FRONT_FLOOR_ANGLE); // 122
        arm.telescopeToSetpoint(0);
      } else if (position == SetPoints.ARM.BACK_FLOOR) { //9
        arm.setShoulderSetpoint(SetPoints.SHOULDER_BACK_FLOOR_ANGLE);
        arm.telescopeToSetpoint(0);
      } else if (position == SetPoints.ARM.BACK_MID) { //8
        arm.setShoulderSetpoint(midBackShoulderSetPoint);
        arm.telescopeToSetpoint(midTeleSetPoint);
      } else if (position == SetPoints.ARM.FRONT_HIGH) { //3
        arm.setShoulderSetpoint(highFrontShoulderSetPoint);
        arm.telescopeToSetpoint(highTeleSetPoint);
      } else if (position == SetPoints.ARM.BACK_HIGH) { //7
        arm.setShoulderSetpoint(highBackShoulderSetPoint);
        arm.telescopeToSetpoint(highTeleSetPoint);
      } else if (position == 20) {
        arm.setShoulderSetpoint(SetPoints.SHOULDER_BACK_HP_ANLGE);
        arm.telescopeToSetpoint(SetPoints.TELESCOPE_UP_LENGTH);
      } else if (position == 21) {
        arm.setShoulderSetpoint(SetPoints.SHOULDER_FRONT_HP_ANLGE);
        arm.telescopeToSetpoint(SetPoints.TELESCOPE_UP_LENGTH);
      } else if (position == 22) {
        autoHuman.setState(autoHuman.getDefaultState());
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
    NtHelper.setString("/robot/autoScore/position", "right");
    
  }
  
  public void resetGyroFromDashboard() {
    boolean isTrue = NtHelper.getBoolean("/dashboard/gyroIsResetting", false);
    if (isTrue) {
      m_drive.resetAngle();
    }
  }
  
  @Override
  public void teleopPeriodic() {
    prevAutoHuman = isAutoHuman;
    isAutoHuman = m_controller.getPOV() == 180;
    boolean isAutoHumanPressed = !prevAutoHuman && isAutoHuman;
    boolean isAutoHumanReleased = prevAutoHuman && !isAutoHuman;
    final double kMaxSpeed = 4;
    Robot.m_drive.addVisionMeasurement(photonEstimator.grabLatestEstimatedPose());
    resetGyroFromDashboard();
    
  

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
    if (isAutoHumanPressed) {
      autoHuman.setState(autoHuman.getDefaultState());
      Robot.m_drive.setBrake(true);
    }
    if (!isAutoHumanReleased) {
      Robot.m_drive.setBrake(false);
    }

    if (m_controller.getBackButtonReleased()) {
      Robot.m_drive.setBrake(false);
    }

    if (m_controller.getStartButton()) {
  
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
    } else if (isAutoHuman) {
        autoHuman.run();
    }
    else {
      
      boolean isSlowMode = m_controller.getLeftTriggerAxis() > 0.2;
      double maxSpeed = (isSlowMode ? 2 : kMaxSpeed);
      double maxRotation = (isSlowMode ? Math.PI * 0.7 : Drivetrain.kMaxAngularSpeed);

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

    if (m_controller_right.getBackButton()) {
      boolean cube = NtHelper.getBoolean("/dashboard/arm/isCubes", true);
      NtHelper.setBoolean("/dashboard/arm/isCubes", !cube);
      updateArmSetpoint();
    }

    if (m_controller_right.getStartButton()) {
      arm.setOutakeSpeed(-0.3);
      if (Robot.arm.getShoulderAngle().getDegrees() < 0) {
        Robot.arm.setShoulderSetpoint(SetPoints.SHOULDER_BACK_DUNK_ANGLE);
      } else {
        Robot.arm.setShoulderSetpoint(SetPoints.SHOULDER_FRONT_DUNK_ANGLE);
      }

    }

    
int buttonindex = -1;

    boolean shiftUp = m_controller_right.getLeftTriggerAxis() > 0.2;
    boolean shiftDown = m_controller_right.getRightTriggerAxis() > 0.2;
    
    switch (m_controller.getPOV()) {
      case 180:
      buttonindex = 22;
      break;
    }
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
      case 180:
      if(shiftUp) {
        buttonindex = 20; //back hp
      } else if(shiftDown) {
        buttonindex = 21; //front hp
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


     // Rrrrrrrumblllllleee
     if (HumanPlayerStationDetection.seesTagCloseEnough()) {
      m_controller.setRumble(RumbleType.kBothRumble, 1);
     } else {
      m_controller.setRumble(RumbleType.kBothRumble, 0);
     }
     
  }

  @Override
  public void testPeriodic() {
    // m_controller.setRumble(RumbleType.kBothRumble, 0);
    double robotPosition = 0;
    if(robotPosition == -2){
      Rumble.setRumble(m_controller, Position.FAR_LEFT);
  } else if(robotPosition == -1){
    Rumble.setRumble(m_controller, Position.LEFT);
  } else if(robotPosition == 0){
    Rumble.setRumble(m_controller, Position.BIPARTISAN);
  } else if(robotPosition == 1){
    Rumble.setRumble(m_controller, Position.RIGHT);
  }  else if(robotPosition == 2){
    Rumble.setRumble(m_controller, Position.FAR_RIGHT);
  } 
    NtHelper.getDouble("/test/robotPosition", robotPosition);
    // if (m_controller.getStartButton()) {
    //   arm.resetShoulder();
    // }
    // NtHelper.setDouble("/test/shoulderPosition", arm.getShoulderEncoderPosition());

    // // double manualSpeed = NtHelper.getDouble("/test/speed", 0); // top speed is 3
    // // double manualAngle = NtHelper.getDouble("/test/angle", 0);
    // SwerveModuleState bloB = new SwerveModuleState(manualSpeed,
    //      Rotation2d.fromDegrees(manualAngle));

    // double[] desiredStates = {
    //     bloB.angle.getRadians(),
    //     bloB.speedMetersPerSecond,
    //     bloB.angle.getRadians(),
    //     bloB.speedMetersPerSecond,
    //     bloB.angle.getRadians(),
    //     bloB.speedMetersPerSecond,
    //     bloB.angle.getRadians(),
    //     bloB.speedMetersPerSecond,
    // };

    // NtHelper.setDoubleArray("/swerve/desiredStates", desiredStates);

    // m_drive.m_frontLeft.setDesiredState(bloB);
    // m_drive.m_frontRight.setDesiredState(bloB);
    // m_drive.m_backLeft.setDesiredState(bloB);
    // m_drive.m_backRight.setDesiredState(bloB);

    // if (m_controller.getXButton()) {
    //   arm.extend();
    // } else if (m_controller.getAButton()) { // double check this
    //   arm.retract();
    // } else {
    //   arm.stopTelescopeMotor();
    // }

    
  }

  @Override
  public void testInit() { //test innit
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
