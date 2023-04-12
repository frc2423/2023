package frc.robot.Led;

public class KwarqsLed {
    private LedController ledController = new LedController(64);
    private static double m_rainbowFirstPixelHue = 1;

    public KwarqsLed() {
        ledController.add("yellow", new Yellow());
        ledController.add("purple", new Purple());
        ledController.add("green", new Green());
        ledController.add("dark", new Dark());
    }

    public void disable() {
        ledController.set("dark");
    }

    public void setYellow() {
        ledController.set("yellow");
    } 

    public void setPurple() {
        ledController.set("purple");
    } 

    public void setGreen() {
        ledController.set("green");
    } 

    public void setRainbow()  {
         // For every pixel
    for (var i = 0; i < 64; i++) {
        // Calculate the hue - hue is easier for rainbows because the color
        // shape is a circle so only one value needs to precess
        final var hue = (m_rainbowFirstPixelHue + (i * 180 / 64)) % 180;
        // Set the value
        int intValueHue = (int) hue;
        ledBuffer.setHSV(i, intValueHue, 255, 128);
      }
      // Increase by to make the rainbow "move"
      m_rainbowFirstPixelHue += 3;
      // Check bounds
      m_rainbowFirstPixelHue %= 180;
    }

    public void run() {
        ledController.run();
    }
}