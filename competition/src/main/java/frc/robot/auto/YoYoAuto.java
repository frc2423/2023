package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.constants.SetPoints;
import frc.robot.util.NtHelper;
import frc.robot.util.stateMachine.State;
import frc.robot.util.stateMachine.StateContext;
import frc.robot.util.stateMachine.StateMachine;

public class YoYoAuto extends StateMachine { //(YoYoYes)

    private Pose2d gridPose;
    private Pose2d gridEndPose;
    private Pose2d gpPose;

public boolean isWallSide = false;

    public YoYoAuto() {
        super("Score");
        NtHelper.setBoolean("/auto/isWallSide", false);
        Robot.arm.beltStop();
            gridPose =  Waypoints.BLUE_GRID_1;
            gridEndPose = Waypoints.BLUE_GRID_2;
            gpPose = Waypoints.BLUE_GP_1.transformBy(new Transform2d(new Translation2d(-.3,0), new Rotation2d()));
    }

    @State(name = "Score")
    public void taxiRun(StateContext ctx) {
        SwerveModuleState zeroState = new SwerveModuleState(0,
                Rotation2d.fromDegrees(0));

        Robot.m_drive.m_frontLeft.setDesiredState(zeroState);
        Robot.m_drive.m_frontRight.setDesiredState(zeroState);
        Robot.m_drive.m_backLeft.setDesiredState(zeroState);
        Robot.m_drive.m_backRight.setDesiredState(zeroState);
        if (ctx.isInit()) {
        isWallSide = NtHelper.getBoolean("/auto/isWallSide", false);
        if (!isWallSide) {
            gridPose =  Waypoints.BLUE_GRID_9;
            gridEndPose = Waypoints.BLUE_GRID_8;
            gpPose = Waypoints.BLUE_GP_4;//.transformBy(new Transform2d(new Translation2d(-.3,0), new Rotation2d()));
        }
        }
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
            Robot.trajectories.setNewTrajectoryGroup(gridPose, gpPose, false); 
            Robot.trajectories.resetOdometry();
            setState("Move");
        }
    }

    @State(name = "Move")
    public void move(StateContext ctx) {
        if (ctx.getTime() > 1) {
        Robot.trajectories.follow_current_path();
        }
        if (ctx.getTime() > .5) {
            Robot.arm.setShoulderSetpoint(SetPoints.SHOULDER_UP_ANGLE);
            Robot.arm.telescopeToSetpoint(0);
        }
        if (Robot.trajectories.isFinished()) {
            setState("Stop");
        }
    }

    @State(name = "Stop")
    public void Stop(StateContext ctx) {
        Robot.m_drive.drive(0, 0, 0, false);
        setState("Acquire");

    }

    @State(name = "Acquire")
    public void Acquire(StateContext ctx) {
        /*
         * arm to forward floor
         * move belt
         * move forward for 1/2 second
         * 
         */
        Robot.arm.setShoulderSetpoint(SetPoints.SHOULDER_FRONT_FLOOR_ANGLE);
        Robot.arm.telescopeToSetpoint(0);
        Robot.arm.intakeBelt();

        Robot.m_drive.drive(0.25, 0, 0, false); //edit this?
        if (ctx.getTime() > 2) {//adjust time here too?
            Robot.arm.beltStop();
            Robot.arm.setShoulderSetpoint(new Rotation2d(Units.degreesToRadians(0)));
            Robot.arm.telescopeToSetpoint(0);
            setState("Yo2");
            // Robot.trajectories.setNewTrajectoryGroup("SecondYo", true);
            Robot.trajectories.setNewTrajectoryGroup(gpPose, gridEndPose, true); 
        }

    }

    @State(name = "Yo2")
    public void yo2(StateContext ctx) {
        // head the back
        Robot.trajectories.follow_current_path();
        Robot.arm.setShoulderSetpoint(SetPoints.SHOULDER_BACK_MID_CUBE_ANGLE);
        Robot.arm.telescopeToSetpoint(0);
        if (Robot.trajectories.isFinished()) {
            setState("TryScoreMid"); //originally TryScoreLow, now after a trajectory leading to the middle grid
        }
    }

    @State(name = "TryScoreMid")
    public void tryScoreMid(StateContext ctx) {
        // try score mid :P
        Robot.arm.setShoulderSetpoint(SetPoints.SHOULDER_BACK_MID_CUBE_ANGLE);
        Robot.arm.telescopeToSetpoint(0);
        Robot.m_drive.drive(0, 0, 0, false);

        if (ctx.getTime() > .2) { //time?
           setState("TryScoreMidPart2");
        }
    }

    @State(name = "TryScoreMidPart2")
    public void tryScoreMid2(StateContext ctx) {
        // try score mid :P
        Robot.arm.outtakeBeltCube();

        if (ctx.getTime() > 1) { //time?
            Robot.arm.beltStop();
            return;
        }
    }
}
