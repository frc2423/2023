package frc.robot.devices;

/**
 * This is an interface that gurantees both the AbsoluteEncoder and RelativeEncoder classes use every
 * function within the interface
 */
public interface IEncoder {

    double getVelocityConversionFactor();

    double getVelocity();

    double getPosition();

    double getPositionConversionFactor();

    void setPosition(double distance);

    void setPositionConversionFactor(double factor);

    void setVelocityConversionFactor(double d);

}
