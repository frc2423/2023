package frc.robot.auto;

import frc.robot.util.stateMachine.StateContext;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Robot;
import frc.robot.constants.ArmPosition;
import frc.robot.constants.SetPoints;
import frc.robot.constants.StateConstants;
import frc.robot.util.NtHelper;
import frc.robot.util.stateMachine.State;
import frc.robot.util.stateMachine.StateMachine;

public class BasedAuto extends StateMachine {
    String position = new String();
    //fix hard coded stuff please

    public BasedAuto(){
        super("Score");
        Robot.arm.beltStop();
        position = NtHelper.getString("/dashboard/autoArm", "low"); //from nt
    }

    @State(name = "Score")
    public void taxiRun(StateContext ctx) {
        SwerveModuleState zeroState = new SwerveModuleState(0,
                Rotation2d.fromDegrees(0));

        Robot.m_drive.m_frontLeft.setDesiredState(zeroState);
        Robot.m_drive.m_frontRight.setDesiredState(zeroState);
        Robot.m_drive.m_backLeft.setDesiredState(zeroState);
        Robot.m_drive.m_backRight.setDesiredState(zeroState);

        if (position.equals(ArmPosition.MID)){
            Robot.arm.setShoulderSetpoint(SetPoints.SHOULDER_BACK_MID_CONE_ANGLE);
            Robot.arm.telescopeToSetpoint(SetPoints.TELESCOPE_MID_CONE_LENGTH);
        } else if (position.equals(ArmPosition.LOW)) {
            Robot.arm.setShoulderSetpoint(SetPoints.SHOULDER_BACK_FLOOR_ANGLE);
            Robot.arm.telescopeToSetpoint(SetPoints.TELESCOPE_BACK_FLOOR_LENGTH);
        }

        if (ctx.getTime() > 1) { //probably want to reduce the time
            setState("Dunk");
        }
    }
    @State(name = "Dunk")
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
            setState("Stahp");
        }
    }

    @State(name = "Stahp")
    public void stahp(StateContext ctx){
        Robot.m_drive.drive(0, 0, 0, false);

    }
}
