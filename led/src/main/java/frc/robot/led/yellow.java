package frc.robot.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

public class yellow implements Led {
    private Timer timer = new Timer();

    public void start(AddressableLEDBuffer buffer, int length) {
        timer.start();
        timer.reset();
    }

    public void run(AddressableLEDBuffer buffer, int length) {
        System.out.println("yellow");
        for (var i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 255,255,0);
        }
    }

    public void end(AddressableLEDBuffer buffer, int length) {
        timer.stop();
    }
}