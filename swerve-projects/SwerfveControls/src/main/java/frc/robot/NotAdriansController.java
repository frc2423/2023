package frc.robot;

import edu.wpi.first.wpilibj.XboxController;


public class NotAdriansController implements Controller {

    public XboxController myController = new XboxController(0);

    @Override
    public double[] getXYValue() {
        double[] blah = new double[2];
        return blah;
    }

    @Override
    public double[] getAngularValue() {
        // TODO Auto-generated method stub
        double[] blah = new double[2];
        return blah;
    }

    @Override
    public boolean getAButton() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public double[] getDeadband() {
        // TODO Auto-generated method stub
        double[] blah = new double[2];
        return blah;
    }
    
}