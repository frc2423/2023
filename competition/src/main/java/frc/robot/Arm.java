package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.util.NtHelper;

public class Arm {
    private PWMSparkMax gripperMotor;
    private NeoMotor telescopeMotor;
    private NeoMotor shoulderMotor;
    private static final int GRIPPER_MOTOR_PWM_PORT = 0;
    private static final double GRIPPER_OPEN_MOTOR_POWER = 0.5;
    private static final double GRIPPER_CLOSE_MOTOR_POWER = -GRIPPER_OPEN_MOTOR_POWER;
    private static final int TELESCOPE_MOTOR_CAN_BUS_PORT = 0;
    private static final double TELESCOPE_EXTENSION_POWER = 0.5;
    private static final double TELESCOPE_RETRACTION_POWER = -TELESCOPE_EXTENSION_POWER;
    private static final double SHOULDER_FORWARD_POWER = 0.15;
    private static final double SHOULDER_BACKWARD_POWER = -SHOULDER_FORWARD_POWER;
    public static final double DISTANCE = 0;
    private double SHOULDER_CONVERSION_FACTOR = 1; //Calculate later
    private double SHOULDER_MINIMUM = -4; //calculate later ;)
    private double SHOULDER_MAXIMUM = 4; //calculate later :)
    private double TELESCOPE_MINIMUM = 0; //calculate later
    private double TELESCOPE_MAXIMUM = 10; //calcate later

    public Arm() {
        //constructs stuff
        gripperMotor = new PWMSparkMax(GRIPPER_MOTOR_PWM_PORT);
        telescopeMotor = new NeoMotor(TELESCOPE_MOTOR_CAN_BUS_PORT,false);
        shoulderMotor = new NeoMotor(15, true);
        shoulderMotor.setConversionFactor(SHOULDER_CONVERSION_FACTOR);
    }

    public void extend() { //arm telescopes out
        //limit extension distance
        NtHelper.setBoolean("/test/telescope",true);

        if(telescopeMotor.getDistance() >= TELESCOPE_MAXIMUM) {
            telescopeMotor.setPercent(0); 
        } else {
            telescopeMotor.setPercent(TELESCOPE_EXTENSION_POWER);
        }
    }

    public void retract() { //arm un-telescopes
        //limit retraction distance
        NtHelper.setBoolean("/test/telescope",false);

        if(telescopeMotor.getDistance() <= TELESCOPE_MINIMUM){
            telescopeMotor.setPercent(0); 
        } else {
            telescopeMotor.setPercent(TELESCOPE_RETRACTION_POWER);
        }
    }

    public void telescopeToSetpoint(double meters) {
        //setpoint is in meters instead of inches or an encoder value (subject to change)
        if (telescopeMotor.getDistance() >= TELESCOPE_MINIMUM && telescopeMotor.getDistance() <= TELESCOPE_MAXIMUM) {
            telescopeMotor.setPercent(0); 
        } else {
            telescopeMotor.setDistance(meters);
        }
    }

    public void shoulderForward() {
        if (shoulderMotor.getDistance() >= SHOULDER_MAXIMUM) {
            NtHelper.setDouble("/robot/shoulder/speed", 0);

            shoulderMotor.setPercent(0);
        } else{
            NtHelper.setDouble("/robot/shoulder/speed", SHOULDER_FORWARD_POWER);
            shoulderMotor.setPercent(SHOULDER_FORWARD_POWER);
        }
        //moves the shoulder angle towards the front of the robot
        //limit shoulder angle
    }

    public void shoulderBack() {
        if (shoulderMotor.getDistance() <= SHOULDER_MAXIMUM) {
            shoulderMotor.setPercent(0);
            NtHelper.setDouble("/robot/shoulder/speed", 0);
        } else{
            shoulderMotor.setPercent(SHOULDER_BACKWARD_POWER);
            NtHelper.setDouble("/robot/shoulder/speed", SHOULDER_BACKWARD_POWER);
        }
        //moves the shoulder angle towards the back of the robot
        //limit shoulder angle
    }

    public void shoulderSetpoint(Rotation2d shoulderAngle) {
        if(shoulderMotor.getDistance() <= SHOULDER_MAXIMUM && shoulderMotor.getDistance() >= SHOULDER_MINIMUM) {
            shoulderMotor.setDistance(shoulderAngle.getRadians());
        } else{
            shoulderMotor.setPercent(0);
        }
        
    }

    public void openGripper() {
        //pneumatics (no input)
    }

    public void closeGripper() {
        //pneumatics (no input)
    }

    public void intakeBelt() {
        //sets belt speed to # > 0
    }

    public void outtakeBelt() {
        //sets belt speed to # < 0
    }

    public void beltStop() {
        //set belt speed to 0
    }

    public void setBeltSpeed(double beltSpeed) {
        //sets constants for belt speeds for intakeBelt, outtakeBelt, and beltStop
    }
    
    public double getTelescopePosition() {
        return telescopeMotor.getDistance();
    }

    public Rotation2d getShoulderAngle() { //may update later
        return new Rotation2d(shoulderMotor.getDistance());
    }
}
