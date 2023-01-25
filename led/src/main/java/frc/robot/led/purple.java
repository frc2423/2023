package frc.robot.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

public class purple implements Led {
    private Timer timer = new Timer();

    public void start(AddressableLEDBuffer buffer, int length) {
        timer.start();
        timer.reset();
    }

    public void run(AddressableLEDBuffer buffer, int length) {
        for (var i = 0; i < buffer.getLength(); i++) {
            buffer.setHSV(i, 255,75,54);
        }
    }

    public void end(AddressableLEDBuffer buffer, int length) {
        timer.stop();
    }
}