// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {
  private final Spark m_left1Motor = new Spark(3);
  private final Spark m_left2Motor = new Spark(4);
  private final Spark m_right1Motor = new Spark(1);
  private final Spark m_right2Motor = new Spark(2);
  private final MotorControllerGroup leftMotors = new MotorControllerGroup(m_left1Motor, m_left2Motor);
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(m_right1Motor, m_right2Motor);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(leftMotors, rightMotors);
  private final XboxController m_controller = new XboxController(0);

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    rightMotors.setInverted(false);
    m_right2Motor.setInverted(false);
    m_right1Motor.setInverted(true);

  }

  @Override
  public void teleopPeriodic() {
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.
    m_robotDrive.arcadeDrive(-m_controller.getLeftY(), -m_controller.getRightX());
  }
}
