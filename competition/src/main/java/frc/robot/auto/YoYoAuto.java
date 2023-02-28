package frc.robot.auto;

import frc.robot.util.stateMachine.StateContext;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.util.stateMachine.State;
import frc.robot.util.stateMachine.StateMachine;

public class YoYoAuto extends StateMachine {

    public YoYoAuto() {
        super("Score");
        Robot.arm.beltStop();
    }

    @State(name = "Score")
    public void taxiRun(StateContext ctx) {
        Robot.arm.setShoulderSetpoint(new Rotation2d(Units.degreesToRadians(-67)));
        Robot.arm.telescopeToSetpoint(35);

        if (ctx.getTime() > 2) {
            setState("Spit");
        }
    }

    @State(name = "Spit")
    public void spit(StateContext ctx) {
        Robot.arm.outtakeBelt();
        if (ctx.getTime() > 1) {
            setState("Moove");
            Robot.arm.setShoulderSetpoint(new Rotation2d(Units.degreesToRadians(0)));
            Robot.arm.telescopeToSetpoint(0);
            Robot.arm.beltStop();
            Robot.trajectories.setNewTrajectoryGroup("FirstYo");
            Robot.trajectories.resetOdometry();
        }
    }

    @State(name = "Moove")
    public void moove(StateContext ctx) {

        Robot.trajectories.follow_current_path();
        if (Robot.trajectories.isFinished()) {
            setState("Stahp");
        }

    }

    @State(name = "Stahp")
    public void stahp(StateContext ctx) {
        Robot.m_drive.drive(0, 0, 0, false);
        setState("Akwire");

    }

    @State(name = "Akwire")
    public void akwire(StateContext ctx) {
        /*
         * arm to forward floor
         * move belt
         * move forward for 1/2 second
         * 
         */
        Robot.arm.setShoulderSetpoint(new Rotation2d(Units.degreesToRadians(110)));
        Robot.arm.telescopeToSetpoint(10);
        Robot.arm.intakeBelt();

        if (Robot.arm.getShoulderAngle().getDegrees() < 113 && Robot.arm.getShoulderAngle().getDegrees() > 107) {
            Robot.m_drive.drive(0.1, 0, 0, false);
            if (ctx.getTime() > 2) {
                Robot.arm.beltStop();
                Robot.arm.setShoulderSetpoint(new Rotation2d(Units.degreesToRadians(0)));
                Robot.arm.telescopeToSetpoint(0);
                setState("Yo2");
                Robot.trajectories.setNewTrajectoryGroup("SecondYo", true);
            }
        } else {
            Robot.m_drive.drive(0, 0, 0, false);

        }
    }

    @State(name = "Yo2")
    public void yo2(StateContext ctx) {
        // head the back
        Robot.trajectories.follow_current_path();
        if (Robot.trajectories.isFinished()) {
            setState("TryScoreLow");
        }
    }

    @State(name = "TryScoreLow")
    public void tryScoreLow(StateContext ctx) {
        // try score low :P
        Robot.arm.setShoulderSetpoint(new Rotation2d(Units.degreesToRadians(-110)));
        Robot.arm.telescopeToSetpoint(10);
        Robot.m_drive.drive(0, 0, 0, false);
        if (ctx.getTime() > 1) {
            Robot.arm.outtakeBelt();
        }
    }
}
