package frc.robot.auto;

import frc.robot.util.stateMachine.StateContext;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.util.stateMachine.State;
import frc.robot.util.stateMachine.StateMachine;

public class BalanceAuto extends StateMachine{
    public BalanceAuto(){
        super("Score");
        Robot.arm.beltStop();
    }
    @State(name = "Score")
    public void taxiRun(StateContext ctx){
        Robot.arm.setShoulderSetpoint(new Rotation2d(Units.degreesToRadians(-67)));
        Robot.arm.telescopeToSetpoint(51);

        if (ctx.getTime() > 2){
            setState("Spit");
        }
    }

    @State(name = "Spit")
    public void spit(StateContext ctx){
        Robot.arm.outtakeBelt();
        if (ctx.getTime() > 1) {
            setState("Moove");
        }
    }

    @State(name = "Moove")
    public void moove(StateContext ctx){
        Robot.arm.beltStop();
        Robot.m_drive.drive(0.5, 0, 0, false);
        if (ctx.getTime() > 4) {
            setState("Stop");
        }
    }

    @State(name = "Stop")
    public void Stop(StateContext ctx){
        if (ctx.isInit()){
            Robot.m_drive.drive(0, 0.3, 0, false);
        } else if (ctx.getTime() > 0.25){
        Robot.m_drive.drive(0, 0, 0, false);
        }
    }
}
