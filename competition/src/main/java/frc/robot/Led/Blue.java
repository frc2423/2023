package frc.robot.Led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class Blue implements Led{
    public void start(AddressableLEDBuffer buffer, int length) {
    }

    public void run(AddressableLEDBuffer buffer, int length) {
        for (var i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 00, 0, 200);
        }
    }

    public void end(AddressableLEDBuffer buffer, int length) {

    }
}
