package frc.robot.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

public class Snake implements Led {

    private int currentLed = 0;
    private Timer timer = new Timer();

    public void start(AddressableLEDBuffer buffer, int length) {
        currentLed = 0;
        timer.start();
        timer.reset();
    }

    public void run(AddressableLEDBuffer buffer, int length) {
        if (currentLed < length && timer.get() > .5) {
            currentLed++;
            buffer.setRGB(currentLed, 0, 50, 50);
            timer.reset();
        }
    }

    public void end(AddressableLEDBuffer buffer, int length) {
        timer.stop();
    }
}
