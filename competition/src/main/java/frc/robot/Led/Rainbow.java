package frc.robot.Led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class Rainbow implements Led{
    public void start(AddressableLEDBuffer buffer, int length) {
    }
    
    private static double m_rainbowFirstPixelHue = 1;

    public void run(AddressableLEDBuffer buffer, int length) {
         // For every pixel
    for (var i = 0; i < LedController.ledBuffer.getLength(); i++) {
        // Calculate the hue - hue is easier for rainbows because the color
        // shape is a circle so only one value needs to precess
        final var hue = (m_rainbowFirstPixelHue + (i * 180 / LedController.ledBuffer.getLength())) % 180;
        // Set the value
        int intValueHue = (int) hue;
        LedController.ledBuffer.setHSV(i, intValueHue, 255, 200);
      }
      // Increase by to make the rainbow "move"
      m_rainbowFirstPixelHue += 3;
      // Check bounds
      m_rainbowFirstPixelHue %= 180;
    }

    public void end(AddressableLEDBuffer buffer, int length) {

    }
}
