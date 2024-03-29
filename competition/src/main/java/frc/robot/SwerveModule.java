// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.util.NtHelper;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;                               
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;


public class SwerveModule {
  // Constants
  private static final double kWheelRadius = Units.inchesToMeters(1.5); // We know this
  private static final double gearRatio = RobotBase.isSimulation() ? 1 : 5.08;
  private static final int kEncoderResolution = 1; // 4096;
  private final DrivetrainRunnable drivetrainRunnable;
  private final Notifier drivetrainNotifier;

  // State from robot logic
  private double driveMotorVoltage = 0;
  private double turnMotorVoltage = 0;

  // State coming from external or simulated devices
  private double driveEncoderRate = 0;
  private double driveEncoderDistance = 0;
  private double turnEncoderRate = 0;
  private double turnEncoderDistance = 0;
  private String name;

  /*
   * private static final double kModuleMaxAngularVelocity =
   * Drivetrain.kMaxAngularSpeed;
   * private static final double kModuleMaxAngularAcceleration =
   * 2 * Math.PI; // radians per second squared
   */

  public final NeoMotor m_driveMotor;
  public final NeoMotor m_turningMotor;

  private final FlywheelSim m_driveSim = new FlywheelSim(DCMotor.getNEO(1), 6.75, 0.025);
  private final FlywheelSim m_turnSim = new FlywheelSim(DCMotor.getNEO(1), 150.0 / 7.0, 0.004096955);

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(0, 0, 0); // for sim use .5

  //Very monsterously massive correct Gains are for example purposes only - must be determined for your own robot!
  private ProfiledPIDController turningPIDController = new ProfiledPIDController( RobotBase.isSimulation() ? 23 : 4, 0, 0.2, new TrapezoidProfile.Constraints(Math.PI * 6, Math.PI * 12));

  private double ks = RobotBase.isSimulation() ? 0.025 : .165;
  private double kv = RobotBase.isSimulation() ? 0.075 : 2.7;

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(ks, kv);


  private double turnKs =  0.16;
  private double turnKv = 0.45;

  private final SimpleMotorFeedforward turnFeedForward = new SimpleMotorFeedforward(turnKs, turnKv); //AHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH

  private final boolean invertDriveEncoderRate;
  private final boolean invertDriveEncoderDistance;

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
   * and turning encoder.
   *
   * @param driveid CAN ID for the drive motor.
   * @param turnid  CAN ID for the turning motor.
   */
  public SwerveModule(int driveid, int turnid, String name, boolean swerveInvert, boolean invertDriveEncoderRate) {
    this.name = name; //:P
    m_driveMotor = new NeoMotor(driveid, false);
    m_turningMotor = new NeoMotor(turnid, true);
    drivetrainRunnable = new DrivetrainRunnable(m_driveMotor, m_turningMotor);
    drivetrainNotifier = new Notifier(drivetrainRunnable); 
    drivetrainNotifier.setName("DrivetrainRunnable");
    drivetrainNotifier.startPeriodic(0.02);
    m_driveMotor.setInverted(swerveInvert);
   

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_driveMotor.setConversionFactor(2 * Math.PI * kWheelRadius / gearRatio);

    // Set the distance (in this case, angle) in radians per pulse for the turning
    // encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    m_turningMotor.setConversionFactor(2 * Math.PI * kEncoderResolution);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    turningPIDController.setTolerance(Math.PI / 180.0);

    this.invertDriveEncoderRate = invertDriveEncoderRate;
    this.invertDriveEncoderDistance = false; // invertDriveEncoderDistance;
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        driveEncoderRate, new Rotation2d(turnEncoderDistance));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {

    return new SwerveModulePosition(
        driveEncoderDistance, new Rotation2d(turnEncoderDistance));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(turnEncoderDistance));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput = m_drivePIDController.calculate(driveEncoderRate, state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.

    driveMotorVoltage = (driveOutput + driveFeedforward);
    double turnVelocity = turningPIDController.calculate(turnEncoderDistance, state.angle.getRadians());
    var atTurnGoal = turningPIDController.atGoal();
    turnMotorVoltage = turnFeedForward.calculate(turnVelocity);
    if (atTurnGoal) {
      turnMotorVoltage = 0;
    }
  }

  public void resetPosition() {
    m_driveMotor.resetEncoder(0);
    m_turningMotor.resetEncoder(0);
    driveEncoderDistance = 0;
    turnEncoderDistance = 0;
  }

  public void updateSimulation(double dtSeconds) {
    // Update simulation inputs
    m_driveSim.setInputVoltage(driveMotorVoltage);
    m_turnSim.setInputVoltage(turnMotorVoltage);
    // Move simulation forward dt seconds
    m_driveSim.update(dtSeconds);
    m_turnSim.update(dtSeconds);
    // Get state from simulation devices
    var encoderRateSign = invertDriveEncoderRate ? -1 : 1;
    driveEncoderRate = m_driveSim.getAngularVelocityRadPerSec() * encoderRateSign;
    driveEncoderDistance = (driveEncoderDistance
        + (m_driveSim.getAngularVelocityRadPerSec() * encoderRateSign * dtSeconds));

    turnEncoderRate = m_turnSim.getAngularVelocityRadPerSec();
    turnEncoderDistance = (turnEncoderDistance + (m_turnSim.getAngularVelocityRadPerSec() * dtSeconds));
  }

  public double getDistance() {
    return m_driveMotor.getDistance();
  }

  public void updateReal() {
    // Update real robot inputs
    m_driveMotor.setPercent(driveMotorVoltage / RobotController.getBatteryVoltage());
    m_turningMotor.setPercent((turnMotorVoltage) / RobotController.getBatteryVoltage());

    // Update state from actual devices
    var encoderRateSign = invertDriveEncoderRate ? -1 : 1;
    var encoderDistanceSign = invertDriveEncoderDistance ? -1 : 1;

    driveEncoderRate = drivetrainRunnable.getDriveMotorSpeed() * encoderRateSign;
    driveEncoderDistance = drivetrainRunnable.getDriveMotorDistance() * encoderDistanceSign;
    turnEncoderRate = drivetrainRunnable.getTurningMotorSpeed();
    turnEncoderDistance = (RobotBase.isSimulation() ? 1 : -1) * drivetrainRunnable.getTurningMotorDistance();

  }

  public void setBrake(boolean brake) {
    m_driveMotor.setBrakeMode(brake);
  }

  public void update() {
    if (RobotBase.isSimulation()) {
      updateSimulation(0.02);
    } else {
      updateReal();
    }
  }


  public double getTurnEncoderRate() {
    return turnEncoderRate;
  }

}
