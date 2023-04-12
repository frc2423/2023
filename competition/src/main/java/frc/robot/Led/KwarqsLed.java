package frc.robot.Led;

public class KwarqsLed {
    private LedController ledController = new LedController(64);
    

    public KwarqsLed() {
        ledController.add("yellow", new Yellow());
        ledController.add("purple", new Purple());
        ledController.add("green", new Green());
        ledController.add("dark", new Dark());
        ledController.add("rainbow", new Rainbow());
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
       ledController.set("rainbow");
    }

    public void run() {
        ledController.run();
    }
}