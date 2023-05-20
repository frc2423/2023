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
        // Can't really discern difference between left/right rumble, focus on rumbling when it is centered next time
        double NtLeftRumble = NtHelper.getDouble("/test/LeftRumble", 0);
        double NtBothRumble = NtHelper.getDouble("/test/BothRumble", 0);
        double NtRightRumble = NtHelper.getDouble("/test/RightRumble", 0);
        NtHelper.getDouble("/test/BothRumble", 0);
        NtHelper.getDouble("/test/RightRumble", 0);
        controller.setRumble(RumbleType.kRightRumble, NtRightRumble);
        // if(position.equals(Position.FAR_LEFT)){
        //     controller.setRumble(RumbleType.kLeftRumble, NtLeftRumble);
        // } else if(position.equals(Position.LEFT)){
        //     controller.setRumble(RumbleType.kLeftRumble, .75);
        // } else if(position.equals(Position.BIPARTISAN)){
        //     controller.setRumble(RumbleType.kBothRumble, 1);
        // } else if(position.equals(Position.RIGHT)){
        //     controller.setRumble(RumbleType.kRightRumble, .75);
        // }  else if(position.equals(Position.FAR_RIGHT)){
        //     controller.setRumble(RumbleType.kRightRumble, .25);
        // } 
            
    }

}