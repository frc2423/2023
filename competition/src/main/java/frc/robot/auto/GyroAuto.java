package frc.robot.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Drivetrain;
import frc.robot.Robot;
import frc.robot.util.LinearScale;
import frc.robot.util.stateMachine.State;
import frc.robot.util.stateMachine.StateContext;
import frc.robot.util.stateMachine.StateMachine;

public class GyroAuto extends StateMachine {
    Timer timer = new Timer();

    public GyroAuto() {
        super("Score");
        Robot.arm.beltStop();
    }

    @State(name = "Score")
    public void taxiRun(StateContext ctx) {

        Robot.arm.setShoulderSetpoint(new Rotation2d(Units.degreesToRadians(-115)));
        Robot.arm.telescopeToSetpoint(0);

        if (ctx.getTime() > 2) {
            setState("Spit");
        }
    }

    @State(name = "Spit")
    public void spit(StateContext ctx) {
        Robot.arm.outtakeBelt();
        if (ctx.getTime() > 1) {
            setState("Start");
        }
    }

    @State(name = "Start")
    public void start(StateContext ctx) {
        Robot.arm.beltStop();
        if (ctx.isInit()) {
            timer.reset();
            timer.start();
        }
        if (ctx.getTime() < 0.2) {
            SwerveModuleState billY = new SwerveModuleState(0,
                    Rotation2d.fromDegrees(0));

            Robot.m_drive.m_frontLeft.setDesiredState(billY);
            Robot.m_drive.m_frontRight.setDesiredState(billY);
            Robot.m_drive.m_backLeft.setDesiredState(billY);
            Robot.m_drive.m_backRight.setDesiredState(billY);

        } else {
            Robot.m_drive.drive(1.05, 0, 0, true);
            Robot.arm.setShoulderSetpoint(new Rotation2d(Units.degreesToRadians(0)));

            if (Robot.m_drive.m_gyro.getPitch() > 3) {
                setState("Reached");
            }

        }
    }

    @State(name = "Reached")
    public void reached(StateContext ctx) {
        Robot.m_drive.drive(1.05, 0, 0, true);
        if (Robot.m_drive.m_gyro.getPitch() < -3) {
            setState("Crossed");
        }
    }

    @State(name = "Crossed")
    public void crossed(StateContext ctx) {
        Robot.m_drive.drive(1.05, 0, 0, true);
        if (Math.abs(Robot.m_drive.m_gyro.getPitch()) < 1.5) {
            setState("Over");
        }
    }

    @State(name = "Over")
    public void over(StateContext ctx) {
        if (ctx.getTime() < .75) {
            Robot.m_drive.drive(1.05, 0, 0, true);
        } else {
            Robot.m_drive.drive(-1.05, 0, 0, true);
            if (Robot.m_drive.m_gyro.getPitch() < -6) {
                setState("KeepGoing");
            }
        }
    }

    @State(name = "KeepGoing")
    public void KeepGoing(StateContext ctx) {
        Robot.m_drive.drive(-1.05, 0, 0, true);
        if (ctx.getTime() > 2.25) {
            setState("Balance");
        }
    }

    double maxSpeed = 0.4;
    double minSpeed = 0;
    double slowDownAngle = 10;
    double stopAngle = 1;

    private final LinearScale balanceScale = new LinearScale(minSpeed, maxSpeed, stopAngle, slowDownAngle);

    @State(name = "Balance")
    public void balance(StateContext ctx) {
        // Robot.m_drive.drive(-0.4, 0, 0, true);
        var speed = balanceScale.calculate(Robot.m_drive.m_gyro.getPitch());
        Robot.m_drive.drive(speed * 0.5, 0, 0, true);
        if (timer.get() > 14.4) {
            setState("stop");
        }
        // if (Robot.m_drive.m_gyro.getPitch() < -3) {
        // Robot.m_drive.drive(-0.4, 0, 0, true);
        // System.out.println("1");
        // }
        // else if(Robot.m_drive.m_gyro.getPitch() > 3) {
        // Robot.m_drive.drive(0.4, 0, 0, true);
        // System.out.println("2");
        // // Robot.m_drive.drive(0, 0, 0, false);
        // // setState("stop");

        // }
        // else {
        // Robot.m_drive.drive(0, 0, 0, false);
        // System.out.println("3");
        // setState("stop");

        // }
    }

    @State(name = "stop")
    public void stop(StateContext ctx) {
        // Robot.m_drive.drive(0, 0, 0, false);

        SwerveModuleState greG = new SwerveModuleState(0,
                Rotation2d.fromDegrees(90));

        Robot.m_drive.m_frontLeft.setDesiredState(greG);
        Robot.m_drive.m_frontRight.setDesiredState(greG);
        Robot.m_drive.m_backLeft.setDesiredState(greG);
        Robot.m_drive.m_backRight.setDesiredState(greG);

        if (ctx.isInit()) {
            Robot.m_drive.setBrake(true);
        }

    }

}