package frc.robot.devices;

import com.revrobotics.CANSparkMax;

public class RelativeEncoder implements IEncoder {

    CANSparkMax motor;
    com.revrobotics.RelativeEncoder encoder;

    public RelativeEncoder(CANSparkMax motor) {
        this.motor = motor;
        encoder = motor.getEncoder();
    }

    @Override
    public double getVelocityConversionFactor() {
        return encoder.getVelocityConversionFactor();
    }

    @Override
    public double getVelocity() {
      return encoder.getVelocity();
    }

    @Override
    public double getPosition() {
        return encoder.getPosition();
    }

    @Override
    public double getPositionConversionFactor() {
        return encoder.getPositionConversionFactor();
    }

    @Override
    public void setPositionConversionFactor(double factor) {
        encoder.setPositionConversionFactor(factor);
        
    }

    @Override
    public void setPosition(double distance) {
        encoder.setPosition(distance);
    }

    @Override
    public void setVelocityConversionFactor(double factor) {
        encoder.setVelocityConversionFactor(factor);
        
    }

    


   
}
