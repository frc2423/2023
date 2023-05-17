package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;



public class Rumble {

    

    public enum Position{
        FAR_LEFT,
        LEFT,
        BIPARTISAN,
        RIGHT,
        FAR_RIGHT
    }

    public static void setRumble(XboxController controller, Position position){
        if(position == Position.FAR_LEFT){
            controller.setRumble(RumbleType.kLeftRumble, .25);
        } else if(position == Position.LEFT){
            controller.setRumble(RumbleType.kLeftRumble, .75);
        }
            
    }

}