package frc.robot;

import frc.robot.util.stateMachine.StateContext;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.CameraConstants;
import frc.robot.util.stateMachine.State;
import frc.robot.util.stateMachine.StateMachine;

public class AutoScoreCube extends StateMachine {
    //dw jacob we made it specific just fpr you
    public AutoScoreCube(){
        super("createPath"); // so rad
    }
    
    public boolean scoringTag() {
        var m_Camera = Robot.m_camera;
        var res = m_Camera.returnCamera().getLatestResult(); 
        var bestTarget = res.getBestTarget();
        var isRed = Alliance.Red.equals(DriverStation.getAlliance());

        if (res.hasTargets()) {
            var targetid = bestTarget.getFiducialId();
    
            if (isRed && (targetid == 1 || targetid == 2 || targetid == 3)){
                return true;
            }
            else if (!isRed && (targetid == 4 || targetid == 5 || targetid == 6)){
                return true; //jejwfnewon
            }
        }

        return false;
        
        //don't want to go to the wrong april tag now do we
    }

    @State(name = "createPath")
    public void createPath(StateContext ctx) {                                                                                               //sus
        //staharting @ current pose ending @ score pose
    }

    @State(name = "followPath") //what?!
    public void followPath(StateContext ctx) {
        //dew it
    }

    @State(name = "stahp")
    public void stahp(StateContext ctx) {
        //when stop :P
    }

    @State(name = "scahr")
    public void scahr() {
        //cahoobz
    }

    // public void sleep() {
    //     //catch some Zs
    // }
    
}
