package frc.robot.util;

/**
 * An object for mapping a range of values to another
 */
public class LinearScale {

  private double minRange;
  private double maxRange;
  private double onTargetMargin;
  private double maxDomain;

  /**
   * @param minRange The minimum value that the object will ever return
   * @param maxRange The maximum value that the object will ever return
   * @param onTargetMargin The maximum error allowed in either direction to consider us at the target value
   * @param maxDomain The error threshold beyond which the min/max value will be returned
   */
  public LinearScale(double minRange, double maxRange, double onTargetMargin, double maxDomain) {
    this.minRange = minRange;
    this.maxRange = maxRange;
    this.onTargetMargin = onTargetMargin;
    this.maxDomain = maxDomain;
  }
  
  /**
   * Takes in an error and returns a value to reduce the error.
   * @param value The error
   * @return The value to provide to the robot.
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
  }
}