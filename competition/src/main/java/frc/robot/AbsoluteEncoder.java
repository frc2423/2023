package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;



public class AbsoluteEncoder implements IEncoder {

    CANSparkMax motor;
    SparkMaxAbsoluteEncoder encoder;

    public AbsoluteEncoder(CANSparkMax motor) {
        this.motor = motor;
        encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
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
        // encoder(distance); [not needed]
    }

    @Override
    public void setVelocityConversionFactor(double factor) {
        encoder.setVelocityConversionFactor(factor / 60);
    }

   
}
