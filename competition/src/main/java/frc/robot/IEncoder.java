package frc.robot;

public interface IEncoder {

    double getVelocityConversionFactor();

    double getVelocity();

    double getPosition();

    double getPositionConversionFactor();

    void setPosition(double distance);

    void setPositionConversionFactor(double factor);

    void setVelocityConversionFactor(double d);

}
