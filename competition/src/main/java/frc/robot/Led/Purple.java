package frc.robot.Led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

public class Purple implements Led {
    Timer timer = new Timer();

    public void start(AddressableLEDBuffer buffer, int length) {
        timer.start();
    }

    public void run(AddressableLEDBuffer buffer, int length) {
        for (var i = 0; i < buffer.getLength(); i++) {
            if(i%2 == 0){
                if(timer.get() < 1){
                    buffer.setRGB(i, 00, 75, 25);//200:0:200
                } else{
                    buffer.setRGB(i, 00,00, 00);
                }
            } else{
                if(timer.get() > 1 && timer.get() < 2){
                    buffer.setRGB(i, 128,64, 00);//200:0:200
                } else{
                    buffer.setRGB(i, 00,00, 00);
                }
            }

            if(timer.get() > 2){
                timer.restart();
            }
        }
//yellow 250, 90, 0 (but divide)
    }

    public void end(AddressableLEDBuffer buffer, int length) {

    }
}
