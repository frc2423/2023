package frc.robot.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;



public class LedAlternate implements Led {
    private Timer timer = new Timer();
    int section = 1;

    public void start(AddressableLEDBuffer buffer, int length) {
        timer.start();
        timer.reset();
    }

    public void run(AddressableLEDBuffer buffer, int length) {
        if (section == 1) {
        for (var i = 0; i < buffer.getLength() / 4; i++) {
            if (timer.get() > 0) {
                buffer.setRGB(i, 0, 255, 0);
            }
            if (timer.get() > 0.9) {
                timer.reset();
                buffer.setRGB(i, 0, 0, 0);
                section = 2;
            }
        }
    }
    if (section == 2) {
        for (var i = buffer.getLength() / 4; i < buffer.getLength() / 2; i++) {
            if (timer.get() > 0) {
                buffer.setRGB(i, 250, 90, 0);
            }
            if (timer.get() > 0.9) {
                timer.reset();
                buffer.setRGB(i, 0, 0, 0);
                section = 3;
            }
        }
        
    }
    if (section == 3) {
        for (var i = buffer.getLength() / 2; i < buffer.getLength() / 1; i++) {
            if (timer.get() > 0) {
                buffer.setRGB(i, 0, 255, 0);
            }
            if (timer.get() > 0.9) {
                timer.reset();
                buffer.setRGB(i, 0, 0, 0);
                section = 1;
                i = 0;
            }
        }
    }
}


    public void end(AddressableLEDBuffer buffer, int length) {

    }
}
