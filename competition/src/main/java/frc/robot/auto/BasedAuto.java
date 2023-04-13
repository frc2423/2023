package frc.robot.auto;

import frc.robot.util.stateMachine.StateContext;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
    public void taxiRun(StateContext ctx) {
        SwerveModuleState zeroState = new SwerveModuleState(0,
                Rotation2d.fromDegrees(0));

        Robot.m_drive.m_frontLeft.setDesiredState(zeroState);
        Robot.m_drive.m_frontRight.setDesiredState(zeroState);
        Robot.m_drive.m_backLeft.setDesiredState(zeroState);
        Robot.m_drive.m_backRight.setDesiredState(zeroState);
       
        Robot.arm.setShoulderSetpoint(SetPoints.SHOULDER_BACK_MID_CONE_ANGLE);
        Robot.arm.telescopeToSetpoint(SetPoints.TELESCOPE_MID_CONE_LENGTH);

        if (ctx.getTime() > 1) { //probably want to reduce the time
            setState("dunk");
        }
    }
    @State(name = "dunk")
    public void dunk(StateContext ctx) {
        Robot.arm.setShoulderSetpoint(SetPoints.SHOULDER_BACK_DUNK_ANGLE);
        if( ctx.getTime() > 1) { //noice
            setState("Spit");
        }

    }


    @State(name = "Spit")
    public void spit(StateContext ctx) {
        Robot.arm.outtakeBeltCone();
        if (ctx.getTime() > .5) {
            Robot.arm.setShoulderSetpoint(SetPoints.SHOULDER_UP_ANGLE);
            Robot.arm.beltStop();
        }
        if (ctx.getTime() > 1) {
            Robot.arm.telescopeToSetpoint(0);
            setState("Move");
        }
    }


    @State(name = "Move")
    public void moove(StateContext ctx){
        Robot.arm.beltStop();
        Robot.m_drive.drive(1, 0, 0, true);
        if (ctx.getTime() > 4) {
            setState("Stop");
        }
    }

    @State(name = "Stop")
    public void Stop(StateContext ctx){
        Robot.m_drive.drive(0, 0, 0, false);

    }
}
