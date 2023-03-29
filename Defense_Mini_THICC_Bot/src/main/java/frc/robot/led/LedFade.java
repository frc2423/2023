package frc.robot.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;



public class LedFade implements Led {
    private Timer timer = new Timer();
    int red = 250;
    int green = 90;
    int blue = 0;
    int change = 1;

    public void start(AddressableLEDBuffer buffer, int length) {
        timer.start();
        timer.reset();
    }

    // private void (double red1, double green1, double blue1, )

    public void run(AddressableLEDBuffer buffer, int length) {

        if (red == 255) {
            change = -1;
        } else if (red == 0) {
            change = 1;
        }
        if (green == 255) {
           change = 1;
        } else if (green == 90) {
           change = -1;
        }

        if (timer.get() > 0.016) {

            red = red + change;

            green = green - change; 

            timer.reset();
        }

        for (var i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, red, green, blue);
        }

    }

    public void end(AddressableLEDBuffer buffer, int length) {

    }
}
