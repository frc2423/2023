package frc.robot.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

public class LedRotate2 implements Led {
    private Timer timer = new Timer();

    public void start(AddressableLEDBuffer buffer, int length) {
        timer.start();
        timer.reset();
    }

    public void run(AddressableLEDBuffer buffer, int length) {
        for (var i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 85, 0, 0);
            if (timer.get() > 4.5) {
                buffer.setRGB(i, 0, 0, 85);
            }
            if (timer.get() > 3.6) {
                buffer.setRGB(i, 85, 85, 0);
            }
            if (timer.get() > 2.7) {
                buffer.setRGB(i, 0, 85, 0);
            }
            if (timer.get() > 1.8) {
                buffer.setRGB(i, 85, 0, 0);
            }
            if (timer.get() > 0.9) {
                buffer.setRGB(i, 85, 85, 0);
            }
            if (timer.get() > 0) {
                buffer.setRGB(i, 0, 85, 0);
                timer.reset();
            }
        }
    }

    public void end(AddressableLEDBuffer buffer, int length) {

    }
}