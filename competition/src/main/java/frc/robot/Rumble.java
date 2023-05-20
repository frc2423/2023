package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.util.NtHelper;



public class Rumble {

    

    public enum Position{
        FAR_LEFT,
        LEFT,
        BIPARTISAN,
        RIGHT,
        FAR_RIGHT, m_controller;
    }

    public static void setRumble(XboxController controller, Position position){
        NtHelper.setDouble("/test/LeftRumble", .25);
        NtHelper.setDouble("/test/BothRumble", .25);
        NtHelper.setDouble("/test/RightRumble", .25);
        if(position.equals(Position.FAR_LEFT)){
            controller.setRumble(RumbleType.kLeftRumble, .25);
        } else if(position.equals(Position.LEFT)){
            controller.setRumble(RumbleType.kLeftRumble, .75);
        } else if(position.equals(Position.BIPARTISAN)){
            controller.setRumble(RumbleType.kBothRumble, 1);
        } else if(position.equals(Position.RIGHT)){
            controller.setRumble(RumbleType.kRightRumble, .75);
        }  else if(position.equals(Position.FAR_RIGHT)){
            controller.setRumble(RumbleType.kRightRumble, .25);
        } 
            
    }

}