package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI.Port;



public class Gyro {
    private double zeroRoll = 0;
    private double zeroPitch = 0;

    private AHRS gyro;

    public Gyro(){
        gyro = new AHRS(Port.kMXP);
    }

    public Rotation2d getRotation2d(){
        return new Rotation2d(Units.degreesToRadians(-getAngle()));
    }

    public Gyro(Port channel){
        gyro = new AHRS(channel);
    }

    public double getPitch() {
        return gyro.getPitch()-zeroPitch;
    }

    public double getRoll() {
        return gyro.getRoll()-zeroRoll;
    }

    public void reset(){
        zeroPitch = gyro.getPitch();
        zeroRoll = gyro.getRoll();
        gyro.reset();
    }

    public double getAngle(){
        return gyro.getAngle();
    }

    public void setAngle(double angle){
        gyro.reset();
        gyro.setAngleAdjustment(angle);
    }

    public double getRate(){
        return gyro.getRate();
    }

    public void calibrate(){
        gyro.calibrate();
    }

    public Rotation2d getRotation (){
        return gyro.getRotation2d();
    }

}