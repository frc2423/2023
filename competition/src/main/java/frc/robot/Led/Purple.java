package frc.robot.Led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

public class Purple implements Led {
    Timer timer = new Timer();
    int x = 0;
    int y = 0;


    public void start(AddressableLEDBuffer buffer, int length) {
        timer.start();
    }

    public void run(AddressableLEDBuffer buffer, int length) {
        for(x=1; x <= 5; x++){
            for(y = 2; y <= 6; y++){
                if(y == 2){
                    buffer.setRGB(coordConversion(x, y), 108, 14, 28);
                }
                if(y == 3){
                    buffer.setRGB(coordConversion(x, y), 116, 67, 10);
                }
                if(y == 4){
                    buffer.setRGB(coordConversion(x, y), 125, 102, 8);
                }
                if(y == 5){
                    buffer.setRGB(coordConversion(x, y), 24, 93,28);
                }
                if(y == 6){
                    buffer.setRGB(coordConversion(x, y), 10, 35, 108);
                }
            }
        }
        

    
        // buffer.setRGB(coordConversion(2, 1), 245,122,7); //divide for actual robot
        // buffer.setRGB(coordConversion(2, 2), 245,122,7);
        // buffer.setRGB(coordConversion(1, 1), 245,122,7); //divide for actual robot
        // buffer.setRGB(coordConversion(1, 2), 245,122,7);

        
        // if (timer.get() > 2){
        //     buffer.setRGB(coordConversion(5, 1), 0,0,20);
        //     buffer.setRGB(coordConversion(6, 1), 0,0,20);
        //     buffer.setRGB(coordConversion(6, 2), 7,90,245);
            
        //     if (timer.get() > 3){
        //     timer.reset();
        //     timer.start();
        //     }
        // }
        // else {
        //     buffer.setRGB(coordConversion(5, 1), 7,90,245);
        //     buffer.setRGB(coordConversion(6, 1), 7,90,245);
           
        // }

        // buffer.setRGB(coordConversion(5, 2), 0,0,255);
        // buffer.setRGB(coordConversion(0, 5), 245,7,7);
        // buffer.setRGB(coordConversion(0, 4), 245,7,7);
        // buffer.setRGB(coordConversion(1, 6), 245,7,7);
        // buffer.setRGB(coordConversion(2, 6), 245,122,7);
        // buffer.setRGB(coordConversion(3, 6), 245,233,7);
        // buffer.setRGB(coordConversion(4, 6), 7,245,98);
        // buffer.setRGB(coordConversion(5, 6), 0,0,255);
        // buffer.setRGB(coordConversion(6, 6), 146,25,225);
        // buffer.setRGB(coordConversion(7, 4), 146,25,225);
        // buffer.setRGB(coordConversion(7, 5), 146,25,225);

        //for (var i = 0; i < buffer.getLength(); i++) {
        //     if(i == 8){
        //         buffer.setRGB(i, 00, 75, 25);//200:0:200
        //     }
        // //     if(i%3 == 0){
        // //         buffer.setRGB(i, 00, 75, 25);//200:0:200
        // //     } else {
        // //         buffer.setRGB(i, 128,64, 00);
        // //     }
        // } 

            // if(timer.get() > 2){
            //     timer.restart();
            // }
            // if(i%2 == 0){
            //     if(timer.get() < 1){
            //         buffer.setRGB(i, 00, 75, 25);//200:0:200
            //     } else{
            //         buffer.setRGB(i, 00,00, 00);
            //     }
            // } else{
            //     if(timer.get() > 1 && timer.get() < 2){
            //         buffer.setRGB(i, 128,64, 00);//200:0:200
            //     } else{
            //         buffer.setRGB(i, 00,00, 00);
            //     }
            // }

            // if(timer.get() > 2){
            //     timer.restart();
            // }
    }
//yellow 250, 90, 0 (but divide)
    
    public int coordConversion(int x, int y) {
        return 8*y + x;
    }

    public void end(AddressableLEDBuffer buffer, int length) {

    }
}
