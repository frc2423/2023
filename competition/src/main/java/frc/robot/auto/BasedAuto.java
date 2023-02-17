package frc.robot.auto;

import frc.robot.util.stateMachine.StateContext;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.util.stateMachine.State;
import frc.robot.util.stateMachine.StateMachine;
public class BasedAuto extends StateMachine {
    
    private double timerDelay = 5;

    public BasedAuto(){
        super("Score");
        Robot.arm.beltStop();
    }

    @State(name = "Wait")
    public void waitRun(StateContext ctx){
        // if (ctx.isInit()) {
        //     Subsystems.follower.setTrajectory("Taxi");   
        //}

        if (ctx.getTime() > timerDelay){
            // setState("Taxi");
        }
    }

    @State(name = "Score")
    public void taxiRun(StateContext ctx){
        Robot.arm.setShoulderSetpoint(new Rotation2d(Units.degreesToRadians(-65)));
        Robot.arm.telescopeToSetpoint(51);

        if (ctx.getTime() > 2){
            setState("Spit");
        }
        // if (ctx.isInit()) {
        //     Subsystems.follower.startFollowing();
        // }
        // Subsystems.follower.follow();
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
        if (ctx.getTime() > 2) {
            setState("Stahp");
        }
    }

    @State(name = "Stahp")
    public void stahp(StateContext ctx){
        Robot.m_drive.drive(0, 0, 0, false);

    }
}
