package frc.robot.auto;

import frc.robot.Robot;
import frc.robot.util.stateMachine.State;
import frc.robot.util.stateMachine.StateContext;
import frc.robot.util.stateMachine.StateMachine;

public class TestAuto extends StateMachine {

    public TestAuto() {
        super("setAuto");
        //TODO Auto-generated constructor stub
    }
    @State(name = "setAuto")
    public void initstuff(StateContext ctx){
        if (ctx.getTime() > 1) {
            System.out.println("got here !!!!!!!!!!!!!!!!!11");

        Robot.trajectories.setNewTrajectoryGroup("ComplexPath");
        setState("move");
        }
    }

    @State(name = "move")
    public void moove(StateContext ctx){
    
        Robot.trajectories.follow_current_path();
        
    }
}

//Do not run over the freshmen (unless it would be really funny lmao)(and the non-programming freshmen are expendable to us)