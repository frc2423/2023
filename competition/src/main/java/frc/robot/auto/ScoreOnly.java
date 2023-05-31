package frc.robot.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.Arm.Position;
import frc.robot.util.stateMachine.State;
import frc.robot.util.stateMachine.StateContext;
import frc.robot.util.stateMachine.StateMachine;

public class ScoreOnly extends StateMachine {
    public ScoreOnly() {
        super("Score");
        //TODO Auto-generated constructor stub
        Robot.arm.beltStop();
    }

    @State(name = "Score")
    public void taxiRun(StateContext ctx){
        var angle = -115; //-67;
        Robot.m_drive.drive(0, 0, 0, false);
        Robot.arm.setPosition(Position.floor);
        if (ctx.getTime() > 1) {
            setState("Spit");
        }
    }

    @State(name = "Spit")
    public void spit(StateContext ctx){
        Robot.arm.outtakeBelt();
        if (ctx.getTime() > 1) {
            setState("Stop");
        }
    }

    @State(name = "Stop")
    public void stop(StateContext ctx) {
        Robot.arm.beltStop();
    }
}
