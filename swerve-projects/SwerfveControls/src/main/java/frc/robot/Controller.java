package frc.robot;

public interface Controller {
    public double[] getXYValue();
    public double[] getAngularValue();
    public boolean getAButton();
    public double[] getDeadband(); //maybe not a double? 
}
