package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.MathUtil;


public class NotAdriansController { // made with anger 

    public XboxController myController = new XboxController(0);

    double deadband = 0.23; // >:c

    public double getForward() {
        return MathUtil.applyDeadband( myController.getLeftY(), deadband); 
    }

    public double getLeft() { // :C
        return MathUtil.applyDeadband( myController.getLeftX(), deadband);

    }

    public double getYaw() { // We do use yaw JACOB :|
        return MathUtil.applyDeadband( myController.getRightX(), deadband);
        
    }
    
    // public static double applyDeadband​(double value, double deadband)
    
}