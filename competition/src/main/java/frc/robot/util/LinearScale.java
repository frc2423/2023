package frc.robot.util;

/**
 * A Rotation object for calculating turn rate values based on an error between an actual and desired rotation
 */
public class LinearScale {

  private double minRange;
  private double maxRange;
  private double onTargetMargin;
  private double maxDomain;


  /**
   * Initializes a new Rotation object
   * @param minRange The minimum rotation speed that the object will ever return
   * @param maxRange The maximum rotation speed that the object will ever return
   * @param onTargetMargin The maximum angle error allowed in either direction to consider us at the target rotation
   * @param maxDomain The angle error threshold beyond which the min/max turn speedo will be returned
   */
  public LinearScale(double minRange, double maxRange, double onTargetMargin, double maxDomain) {
    this.minRange = minRange;
    this.maxRange = maxRange;
    this.onTargetMargin = onTargetMargin;
    this.maxDomain = maxDomain;
  }
  
  /**
   * Takes in an angle error and returns a proper rotation speed to reduce the error.
   * @param value The angle error
   * @return The rotation speed to provide to the robot.
   */
  public double calculate(double value) { 
    
    //the two line bellow are just setting a reverse deadband
    if (value >= maxDomain) {
      return maxRange;
    }
    if (value <= -maxDomain) {
      return -maxRange;
    }
    
    //checks if we are at the target
    if (value >= -onTargetMargin && value <= onTargetMargin) {
      return 0;

    }

    // turn speed = (1 - .1) * (10 - 5) / (15 - 5) + .1 = 0.55
    // turn speed = (1 - .1) * (15 - 5) / (15 - 5) + .1 = 1
    // turn speed = (1 - .1) * (5 - 5) / (10 - 5) + .1 = .1
   
    // turn speed = (1 - .1) * (-10 - 5) / (15 - 5) + .1 = -1.25
    // turn speed = (1 - .1) * (-15 - 5) / (15 - 5) + .1 = -1.7
    // turn speed = (1 - .1) * (-5 - 5) / (10 - 5) + .1 = -1.7


    //Sets the speed depending on the distance of target
    double absValue = Math.abs(value);

    double turnSpeed = (maxRange - minRange) * (absValue - onTargetMargin) / (maxDomain - onTargetMargin) + minRange;
    if (value > onTargetMargin) {
      return turnSpeed;
    }

    return -turnSpeed;
  }

  public boolean isDone(double x){
    return x >= -onTargetMargin && x <= onTargetMargin;
    //return (angle < maxX && angle > minX);
  }
}