package frc.robot;

public interface IEncoder {
    // motor = new CANSparkMax(port, MotorType.kBrushless);
    // motor.encoder(Type.kDutyCycle).getVelocityConversionFactor();

    double getVelocityConversionFactor();

    double getVelocity();

    double getPosition();

    double getPositionConversionFactor();

    void setPosition(double distance);

	void setPositionConversionFactor(double factor);

    void setVelocityConversionFactor(double d);

    


}
