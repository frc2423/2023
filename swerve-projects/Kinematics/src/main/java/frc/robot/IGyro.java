package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IGyro{

    public void reset();

    public Rotation2d getRotation();

    /**
     * Returns angle reported by the gyro. Clockwise means positive angle.
     * 
     * @return angle
     */
    public double getAngle();

    /**
     * Sets angle of the gyro.
     * 
     * @param angle Clockwise means positive angle.
     */
    public void setAngle(double angle);

    public double getRate();

    public void calibrate();

}
