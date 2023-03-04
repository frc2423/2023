package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.Camera;
import frc.robot.util.LinearScale;
import frc.robot.util.NtHelper;

public class AutoAlign {

    private final LinearScale objectalignment = new LinearScale(.1, 1, 5, 15);
    private final LinearScale robotRotate = new LinearScale(Rotation2d.fromDegrees(25).getRadians(), Math.PI,
        Rotation2d.fromDegrees(5).getRadians(),
        Rotation2d.fromDegrees(50).getRadians());
    private final Camera camera = new Camera(null);

    public void frontFloor() {
        Robot.arm.setShoulderSetpoint(new Rotation2d(Units.degreesToRadians(110)));
        Robot.arm.telescopeToSetpoint(10);
      }
    
      public void frontScoreMid() {
        Robot.arm.setShoulderSetpoint(new Rotation2d(Units.degreesToRadians(57))); // 57 but 59 good
        Robot.arm.telescopeToSetpoint(51);
      }
    
      public void autoScore() {
        Robot.m_drive.drive(0, 0, 0, false);
        String AutoPos = NtHelper.getString("/robot/score/AutoPos", "low");
        if (AutoPos.equals("low")) {
          frontFloor();
        } else if (AutoPos.equals("mid")) {
          frontScoreMid();
        } else if (AutoPos.equals("high")) {
          // make a function for high score
        } else {
        }
      }
    
      public void autoAlign() {
        if (camera.seesTarget()) {
          var tapeX = camera.getTapeX() + 8;
          if (!objectalignment.isDone(tapeX)) {
            var speed = objectalignment.calculate(tapeX);
            Robot.m_drive.drive(0, -speed, 0, false);
    
            NtHelper.setDouble("/autoAlign/speed", speed);
            NtHelper.setDouble("/autoAlign/tapeX", tapeX);
          } else {
           // autoScore();
           Robot.m_drive.drive(0, 0, 0, false);
          }
        } else {
          Robot.m_drive.drive(0, 0, 0, false);
        }
      }
    
      public void autoRotate() {
        var angleError = MathUtil.angleModulus(Math.PI - Robot.m_drive.getAngle().getRadians());
        NtHelper.setDouble("/robot/score/angleError", angleError);
        if (!robotRotate.isDone(angleError)) {
          var rotationSpeed = robotRotate.calculate(angleError);
          Robot.m_drive.drive(0, 0, rotationSpeed, false);
          NtHelper.setDouble("/robot/angle/angularpVelocity", Math.toDegrees(rotationSpeed));
    
        } else {
          autoAlign();
          //m_drive.drive(0, 0, 0, false);
        }
      }
}
