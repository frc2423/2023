package frc.robot.led;


import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
public class rainbow implements Led {
    private Timer timer = new Timer();

    public void start(AddressableLEDBuffer buffer, int length) {
        timer.start();
        timer.reset();
    }
    public void run(AddressableLEDBuffer buffer, int length) {
        int ledlength = buffer.getLength();
        System.out.println("yellow");
        for (var i = 0; i < ledlength; i++) {
            
            var percent = (i / ledlength);
            var hueMax = 180;
            var hue = (int) (hueMax * percent);
            buffer.setHSV(i, hue, 255, 150);
        }
    }

    public void end(AddressableLEDBuffer buffer, int length) {
        timer.stop();
    }
}