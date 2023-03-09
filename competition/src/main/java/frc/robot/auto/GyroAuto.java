package frc.robot.auto;

import frc.robot.util.stateMachine.StateContext;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.util.stateMachine.State;
import frc.robot.util.stateMachine.StateMachine;

public class GyroAuto extends StateMachine{
    public GyroAuto(){
        super("Start");
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
            setState("Start");
        }
    }

    @State(name = "Start")
    public void start(StateContext ctx){
        Robot.arm.beltStop();
        Robot.m_drive.drive(0.5, 0, 0, true);
        if (Robot.m_drive.m_gyro.getPitch() > 3) {
            setState("Reached");
        }
    }

    @State(name = "Reached")
    public void reached(StateContext ctx) {
        Robot.m_drive.drive(0.5, 0, 0, true);
        if (Robot.m_drive.m_gyro.getPitch() < -3) {
            setState("Crossed");
        }
    }

    @State(name = "Crossed")
    public void crossed(StateContext ctx) {
        Robot.m_drive.drive(0.5, 0, 0, true);
        if (Math.abs(Robot.m_drive.m_gyro.getPitch()) < 1.5) {
            setState("Over");
        }
    }

    @State(name = "Over")
    public void over(StateContext ctx) {
        Robot.m_drive.drive(-0.5, 0, 0, true);
        if (Robot.m_drive.m_gyro.getPitch() < -3) {
            setState("Balance");
        }
    }

    @State(name = "Balance")
    public void balance(StateContext ctx) {
        // Robot.m_drive.drive(-0.4, 0, 0, true);
        if (Robot.m_drive.m_gyro.getPitch() < -3) {
            Robot.m_drive.drive(-0.4, 0, 0, true);
        }
        else if(Robot.m_drive.m_gyro.getPitch() > 3) {
            Robot.m_drive.drive(0.4, 0, 0, true);
            // Robot.m_drive.drive(0, 0, 0, false);
            // setState("stop");

        }
        else {
            Robot.m_drive.drive(0, 0, 0, false);
            // setState("stop");
            
        }
    }


 @State(name = "stop")
    public void stop(StateContext ctx) {
        Robot.m_drive.drive(0, 0, 0, false);

    }

    
}