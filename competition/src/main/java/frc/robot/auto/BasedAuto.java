package frc.robot.auto;

import frc.robot.util.stateMachine.StateContext;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.constants.SetPoints;
import frc.robot.util.stateMachine.State;
import frc.robot.util.stateMachine.StateMachine;
public class BasedAuto extends StateMachine {
    
    public BasedAuto(){
        super("Score");
        Robot.arm.beltStop();
    }

    @State(name = "Score")
    public void taxiRun(StateContext ctx){
        var angle = -115; //-67;
        Robot.arm.setShoulderSetpoint(SetPoints.SHOULDER_BACK_MID_CONE_ANGLE);
        Robot.arm.telescopeToSetpoint(0); // CUBE

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
        Robot.m_drive.drive(1, 0, 0, false);
        if (ctx.getTime() > 4) {
            setState("Stahp");
        }
    }

    @State(name = "Stahp")
    public void stahp(StateContext ctx){
        Robot.m_drive.drive(0, 0, 0, false);

    }
}
