package frc.robot.devices;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;


/**
 * This is a class repsenting an absolute encoder; An encoder that retains the zero-values of distance and angle
 * every power cycle rather than relative to the robot's position and distance, as opposed to a relative encoder,
 * which resets the locations of its zero-values for distance and position in relation to the robot every power cycle
 */
public class AbsoluteEncoder implements IEncoder {

    CANSparkMax motor;
    SparkMaxAbsoluteEncoder encoder;

    /**
     * Constructs a new absolute encoder for a given motor with an encoder
     * @param motor pertains to the motor with the encoder that will be used
     */
    public AbsoluteEncoder(CANSparkMax motor) {
        this.motor = motor;
        encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
    }

    /**
     * @see {@link com.revrobotics.SparkMaxAbsoluteEncoder#getVelocityConversionFactor()}
     */
    @Override
    public double getVelocityConversionFactor() {
        return encoder.getVelocityConversionFactor();
    }

    /**
     * 
     */
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
        encoder.setVelocityConversionFactor(factor);
    }
}
