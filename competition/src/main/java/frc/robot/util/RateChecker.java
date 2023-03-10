package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;

public class RateChecker {
    Timer timer = new Timer();
    double timePeriod = 0;
    Double newValue = null;
    Double previousValue = null;

    public RateChecker(double timePeriod_hugh) {
        timePeriod = timePeriod_hugh;
    }

    public void update(double value) {
        if (timer.get() > timePeriod) {
            previousValue = newValue;
            newValue = value;
            timer.reset();
        }

    }

    public void startTimer() {
        timer.reset();
        timer.start();
        newValue = null;
        previousValue = null;
    }

    public Double getRate() {
        if (newValue == null || previousValue == null) {
            return null;
        }
        else {
            double differenceValue = newValue - previousValue;
            return differenceValue / timePeriod;
        }
    }
}
