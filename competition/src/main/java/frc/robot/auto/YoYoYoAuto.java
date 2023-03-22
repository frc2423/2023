package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.constants.SetPoints;
import frc.robot.util.stateMachine.State;
import frc.robot.util.stateMachine.StateContext;
import frc.robot.util.stateMachine.StateMachine;

public class YoYoYoAuto extends StateMachine {
    /* Cone mid
     * Cube mid
     * Cube low
     */

    private Pose2d gridPose;
    private Pose2d gridEndPose;
    private Pose2d gpPose;

    public YoYoYoAuto() {
        super("Score");
        Robot.arm.beltStop();
        gridPose =  Waypoints.BLUE_GRID_1;
        gridEndPose = Waypoints.BLUE_GRID_2;
        gpPose = Waypoints.BLUE_GP_1.transformBy(new Transform2d(new Translation2d(-.3,0), new Rotation2d()));
    
    }

    @State(name = "Score")
    public void taxiRun(StateContext ctx) {
        Robot.arm.setShoulderSetpoint(SetPoints.SHOULDER_BACK_MID_CONE_ANGLE);
        Robot.arm.telescopeToSetpoint(SetPoints.TELESCOPE_MID_CONE_LENGTH);

        if (ctx.getTime() > 1) {
            setState("Spit");
        }
    }

    @State(name = "Spit")
    public void spit(StateContext ctx) {
        Robot.arm.outtakeBelt();
        if (ctx.getTime() > .5) {
            setState("Move");
            Robot.arm.setShoulderSetpoint(SetPoints.SHOULDER_UP_ANGLE);
            Robot.arm.telescopeToSetpoint(0);
            Robot.arm.beltStop();
            Robot.trajectories.setNewTrajectoryGroup(Waypoints.BLUE_GRID_1, Waypoints.BLUE_GP_1, false);
            Robot.trajectories.resetOdometry();
        }
    }

    @State(name = "Move")
    public void move(StateContext ctx) {

        Robot.trajectories.follow_current_path();
        // Robot.arm.intakeBelt();
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
        Robot.arm.telescopeToSetpoint(SetPoints.TELESCOPE_FRONT_FLOOR_LENGTH);
        Robot.arm.intakeBelt();

        Robot.m_drive.drive(0.1, 0, 0, false);
        if (ctx.getTime() > 2) {
            Robot.arm.beltStop();
            Robot.arm.setShoulderSetpoint(SetPoints.SHOULDER_UP_ANGLE);
            Robot.arm.telescopeToSetpoint(SetPoints.TELESCOPE_UP_LENGTH);
            setState("Yo2");
            // Robot.trajectories.setNewTrajectoryGroup("SecondYo", true);
            Robot.trajectories.setNewTrajectoryGroup(Waypoints.BLUE_GP_1, Waypoints.BLUE_GRID_1, true);
        }

    }

    @State(name = "Yo2")
    public void yo2(StateContext ctx) {
        // head the back
        Robot.trajectories.follow_current_path();
        Robot.arm.setShoulderSetpoint(SetPoints.SHOULDER_BACK_MID_CUBE_ANGLE);
        Robot.arm.telescopeToSetpoint(SetPoints.TELESCOPE_UP_LENGTH);
        if (Robot.trajectories.isFinished()) {
            setState("TryScoreLow");
        }
    }

    @State(name = "TryScoreLow")
    public void tryScoreLow(StateContext ctx) {
        // try score mid :P
        Robot.arm.setShoulderSetpoint(SetPoints.SHOULDER_BACK_MID_CUBE_ANGLE);
        Robot.arm.telescopeToSetpoint(SetPoints.TELESCOPE_UP_LENGTH);
        Robot.m_drive.drive(0, 0, 0, false);
        Robot.arm.outtakeBelt();

        if (ctx.getTime() > 1) {
            setState("GoBackOut");

        }
    }

    @State(name = "GoBackOut")
    public void GoBackOut(StateContext ctx) {
        if (ctx.isInit()) {
            Robot.trajectories.setNewTrajectoryGroup(Waypoints.BLUE_GRID_1, Waypoints.BLUE_CHARGE_1,
                    Waypoints.BLUE_GP_2, false);
        }
        Robot.trajectories.follow_current_path();
        Robot.arm.setShoulderSetpoint(SetPoints.SHOULDER_UP_ANGLE);
        Robot.arm.telescopeToSetpoint(SetPoints.TELESCOPE_UP_LENGTH);
        if (Robot.trajectories.isFinished()) {
            setState("akwaia2");
        }
    }

    @State(name = "akwaia2")
    public void akwaia2(StateContext ctx) {
        Robot.arm.setShoulderSetpoint(SetPoints.SHOULDER_FRONT_FLOOR_ANGLE);
        Robot.arm.telescopeToSetpoint(SetPoints.TELESCOPE_FRONT_FLOOR_LENGTH);
        Robot.m_drive.drive(0.1, 0, 0, false);
        Robot.arm.intakeBelt();
        if (ctx.getTime() > .5) {
            Robot.arm.beltStop();
            // Robot.arm.setShoulderSetpoint(new Rotation2d(Units.degreesToRadians(0)));
            // Robot.arm.telescopeToSetpoint(0);
            // setState("");
            // Robot.trajectories.setNewTrajectoryGroup("SecondYo", true);
            Robot.arm.setShoulderSetpoint(SetPoints.SHOULDER_UP_ANGLE);
            Robot.arm.telescopeToSetpoint(SetPoints.TELESCOPE_UP_LENGTH);
            Robot.trajectories.setNewTrajectoryGroup(Waypoints.BLUE_GP_2, Waypoints.BLUE_CHARGE_1,
                    Waypoints.BLUE_GRID_2, true);
            setState("Yo3");
        }

    }

    @State(name = "Yo3")
    public void yo3(StateContext ctx) {
        // head the back
        Robot.trajectories.follow_current_path();
        if (Robot.trajectories.isFinished()) {
            setState("TryScoreLow2");
        }
    }

    @State(name = "TryScoreLow2")
    public void tryScoreLow2(StateContext ctx) {
        // try score low :P
        Robot.arm.setShoulderSetpoint(SetPoints.SHOULDER_BACK_FLOOR_ANGLE);
        Robot.arm.telescopeToSetpoint(SetPoints.TELESCOPE_BACK_FLOOR_LENGTH);
        Robot.m_drive.drive(0, 0, 0, false);
        Robot.arm.outtakeBelt();
    }
}
