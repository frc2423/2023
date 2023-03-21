package frc.robot.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.constants.SetPoints;
import frc.robot.util.stateMachine.State;
import frc.robot.util.stateMachine.StateContext;
import frc.robot.util.stateMachine.StateMachine;

public class YoYoAuto extends StateMachine { //(YoYoYes)
    /*Things to fix after testing robot irl:
     * - the scoring for the cone should be extended to actually scor the cone -after fixing:
     *  maybe put arm down a bit before outtaking because is launching cone
     * - the intake should only be when aquiring
     * - when scoring the cube it should outake once at the proper shoulder angle
     * - meantioned later but: do we want to stop a bit earlier for first yo and go 
     * slowly in akwire to actually intake stuff 
     */

    public YoYoAuto() {
        super("Score");
        Robot.arm.beltStop();
    }

    @State(name = "Score")
    public void taxiRun(StateContext ctx) {
        Robot.arm.setShoulderSetpoint(SetPoints.SHOULDER_BACK_MID_ANGLE);
        Robot.arm.telescopeToSetpoint(SetPoints.TELESCOPE_BACK_MID_LENGTH);

        if (ctx.getTime() > 1) {
            setState("Spit");
        }
    }

    @State(name = "Spit")
    public void spit(StateContext ctx) {
        Robot.arm.outtakeBelt();
        if (ctx.getTime() > .5) {
            setState("Move");
            Robot.arm.setShoulderSetpoint(new Rotation2d(Units.degreesToRadians(0)));
            Robot.arm.telescopeToSetpoint(0);
            Robot.arm.beltStop();
            Robot.trajectories.setNewTrajectoryGroup(Waypoints.BLUE_GRID_1, Waypoints.BLUE_GP_1, false);
            Robot.trajectories.resetOdometry();
        }
    }

    @State(name = "Move")
    public void move(StateContext ctx) {

        Robot.trajectories.follow_current_path();
        Robot.arm.setShoulderSetpoint(new Rotation2d(Units.degreesToRadians(0)));
        Robot.arm.telescopeToSetpoint(0);
        Robot.arm.intakeBelt();
        if (Robot.trajectories.isFinished()) {
            setState("Stahp");
        }
    }

    @State(name = "Stahp")
    public void stahp(StateContext ctx) {
        Robot.m_drive.drive(0, 0, 0, false);
        setState("Akwire");

    }

    @State(name = "Akwire")
    public void akwire(StateContext ctx) {
        /*
         * arm to forward floor
         * move belt
         * move forward for 1/2 second
         * 
         */
        Robot.arm.setShoulderSetpoint(SetPoints.SHOULDER_FRONT_FLOOR_ANGLE);
        Robot.arm.telescopeToSetpoint(0);
        Robot.arm.intakeBelt();

        Robot.m_drive.drive(0.1, 0, 0, false);
        if (ctx.getTime() > 2) {
            //TODO(possibly): Add code to move forward at a low speed for a small amount of time
            Robot.arm.beltStop();
            Robot.arm.setShoulderSetpoint(new Rotation2d(Units.degreesToRadians(0)));
            Robot.arm.telescopeToSetpoint(0);
            setState("Yo2");
            // Robot.trajectories.setNewTrajectoryGroup("SecondYo", true);
            Robot.trajectories.setNewTrajectoryGroup(Waypoints.BLUE_GP_1, Waypoints.BLUE_GRID_2, true);
        }

    }

    @State(name = "Yo2")
    public void yo2(StateContext ctx) {
        // head the back
        Robot.trajectories.follow_current_path();
        Robot.arm.setShoulderSetpoint(new Rotation2d(Units.degreesToRadians(0)));
        Robot.arm.telescopeToSetpoint(0);
        if (Robot.trajectories.isFinished()) {
            setState("TryScoreMid"); //originally TryScoreLow, now after a trajectory leading to the middle grid
        }
    }

    @State(name = "TryScoreMid")
    public void tryScoreMid(StateContext ctx) {
        // try score low :P
        Robot.arm.setShoulderSetpoint(SetPoints.SHOULDER_BACK_MID_ANGLE);
        Robot.arm.telescopeToSetpoint(0);
        Robot.m_drive.drive(0, 0, 0, false);
        Robot.arm.outtakeBelt();

        if (ctx.getTime() > 1) {
            Robot.arm.beltStop();
            return;
        }
    }
}
