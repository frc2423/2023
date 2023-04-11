package frc.robot.auto;

import java.nio.file.Path;
import java.util.Set;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.Robot;
import frc.robot.constants.SetPoints;
import frc.robot.util.stateMachine.State;
import frc.robot.util.stateMachine.StateContext;
import frc.robot.util.stateMachine.StateMachine;

public class TwoPeiceAuto extends StateMachine {
    public TwoPeiceAuto() {
        super("Score");
        Robot.arm.beltStop();

    }

    @State(name = "Score")
    public void score(StateContext ctx){
        Robot.arm.setShoulderSetpoint(SetPoints.SHOULDER_BACK_MID_CONE_ANGLE);
        if (ctx.getTime() > 0.3) {
            Robot.arm.telescopeToSetpoint(SetPoints.TELESCOPE_MID_CONE_LENGTH);
        }
        if (ctx.getTime() > 1){ // CHANGE THIS TIME
            setState("dunk");
        }
    }

    @State(name = "dunk")
    public void dunk(StateContext ctx) {
        Robot.arm.setShoulderSetpoint(SetPoints.SHOULDER_BACK_DUNK_ANGLE);
        if( ctx.getTime() > 1) { //noice
            setState("spit");
        }

    }


    @State(name = "spit")
    public void spit(StateContext ctx) {
        Robot.arm.outtakeBeltCone();
        if (ctx.getTime() > .5) {
            Robot.arm.setShoulderSetpoint(SetPoints.SHOULDER_UP_ANGLE);
            Robot.arm.beltStop();
        }
        if (ctx.getTime() > 0.8) {
            Robot.arm.telescopeToSetpoint(0);
            setState("setTrajectory");
        }
    }


    @State(name = "setTrajectory")
    public void setTrajectory(StateContext ctx) {
        System.out.println("setTraj");
        Robot.trajectories.setNewTrajectoryGroup("getFirstGamePeice");
        Robot.trajectories.resetOdometry(); 
        if (ctx.getTime() > 0.01) {
            setState("move");
        }
        
    }   

    @State(name = "move")
    public void move(StateContext ctx) {
        System.out.println("move");
        Robot.trajectories.follow_current_path();
        if (Robot.trajectories.isFinished()) {
            setState("armDown");
        }
    }

    @State(name = "armDown")
    public void armDown(StateContext ctx) {
        System.out.println("armDown");
        Robot.arm.setShoulderSetpoint(SetPoints.SHOULDER_FRONT_FLOOR_ANGLE);
        Robot.arm.intakeBelt();
        if (ctx.getTime() > 1) {//test this time
            Robot.m_drive.drive(1.2, 0, 0, false);
        }
        if (ctx.getTime() > 1) {
            Robot.arm.setShoulderSetpoint(SetPoints.SHOULDER_UP_ANGLE);
            setState("setBackTraj");
            
        }
    }

    @State(name = "setBackTraj")
    public void setBackTraj(StateContext ctx) {
        Robot.m_drive.drive(0,0,0,false);
        Robot.trajectories.setNewTrajectoryGroup("goBackToScore");
        if (ctx.getTime() > 0.2) {
            setState("moveBack");
        }
    }
    
    @State(name = "moveBack")
    public void moveBack(StateContext ctx) {
        System.out.println("MoveBACK");
        Robot.trajectories.follow_current_path();
        if (Robot.trajectories.isFinished()) {
            setState("scoreCube");
        }

    }

    @State(name = "scoreCube")
    public void scoreCube(StateContext ctx) {
        Robot.arm.setShoulderSetpoint(SetPoints.SHOULDER_BACK_MID_CUBE_ANGLE);
        if (ctx.getTime() > 0.4) {
            Robot.arm.outtakeBelt();
        }
    }
}
