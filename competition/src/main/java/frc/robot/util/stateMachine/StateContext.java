package frc.robot.util.stateMachine;

import edu.wpi.first.wpilibj.Timer;

public class StateContext {

    private Timer timer = new Timer();
    private boolean initialized = false;

    public boolean isInit() {
        return !initialized;
    }

    protected void initialize() {
        initialized = true;
        timer.start();
    }

    public double getTime() {
        return timer.get();
    }
}
