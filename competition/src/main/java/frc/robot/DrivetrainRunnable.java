package frc.robot;

import java.util.concurrent.atomic.AtomicReference;

public class DrivetrainRunnable implements Runnable {
    AtomicReference<Double> driveMotorSpeed = new AtomicReference<Double>(0.0);
    AtomicReference<Double> driveMotorDistance = new AtomicReference<Double>(0.0);
    AtomicReference<Double> turningMotorSpeed = new AtomicReference<Double>(0.0);
    AtomicReference<Double> turningMotorDistance = new AtomicReference<Double>(0.0);
    public NeoMotor m_driveMotor;
    public NeoMotor m_turningMotor;
    

    public DrivetrainRunnable(NeoMotor driveMotor, NeoMotor turningMotor) {
        m_driveMotor = driveMotor;
        m_turningMotor = turningMotor;
    }                                                                                                      // hi toba

    @Override
    public void run() {
        driveMotorSpeed.set(m_driveMotor.getSpeed()); 
        driveMotorDistance.set(m_driveMotor.getDistance());
        turningMotorSpeed.set(m_turningMotor.getSpeed());
        turningMotorDistance.set(m_turningMotor.getDistance()); 
    } //get variable name value

    public double getDriveMotorSpeed(){
        return driveMotorSpeed.get();
    }

    public double getDriveMotorDistance(){
        return driveMotorDistance.get();
    }

    public double getTurningMotorSpeed(){
        return turningMotorSpeed.get();
    }
    
    public double getTurningMotorDistance(){
        return turningMotorDistance.get();
    }

}
