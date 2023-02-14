package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.util.NtHelper;


public class Arm {
    private PWMSparkMax gripperMotor;
    private NeoMotor telescopeMotor;
    private NeoMotor shoulderMotor;
    private PWMSparkMax beltoMotor;
    private static final int GRIPPER_MOTOR_PWM_PORT = 0;
    private static final double GRIPPER_OPEN_MOTOR_POWER = 0.5;
    private static final double GRIPPER_CLOSE_MOTOR_POWER = -GRIPPER_OPEN_MOTOR_POWER;
    private static final int TELESCOPE_MOTOR_CAN_BUS_PORT = 9;
    private static final double TELESCOPE_EXTENSION_POWER = 0.25;
    private static final double TELESCOPE_RETRACTION_POWER = -TELESCOPE_EXTENSION_POWER;
    private static final double SHOULDER_FORWARD_POWER = 0.35;
    private static final double SHOULDER_BACKWARD_POWER = -SHOULDER_FORWARD_POWER;
    public static final double DISTANCE = 0;
    private double SHOULDER_CONVERSION_FACTOR = 1; //Calculate later (motor is 80:1)
    private double SHOULDER_MINIMUM = -120; //calculate later ;)
    private double SHOULDER_MAXIMUM = 120; //calculate later :)
    private double TELESCOPE_MINIMUM = 0; 
    private double TELESCOPE_MAXIMUM = 97; 
    private CANCoder shoulderEncoder  = new CANCoder(25);
    CANCoderConfiguration _canCoderConfiguration = new CANCoderConfiguration();
    PIDController shoulder_PID = new PIDController(.01 , 0.00075 , 0);
    private double TELESCOPE_CONVERSION_FACTOR = 1; //gear ratio
    private double currentShoulderAngle = 0;
    private double beltoSpeedo = .25;

    /*
     * TODO:
     *  - check if getting can values from shoulder encoder -> if not reduce speed -> do later 
     *  - sleep (more than 5 hours)
     *  - bleh
     */

    public Arm() {
        //constructs stuff
        // gripperMotor = new PWMSparkMax(GRIPPER_MOTOR_PWM_PORT);
        telescopeMotor = new NeoMotor(TELESCOPE_MOTOR_CAN_BUS_PORT,false);
        shoulderMotor = new NeoMotor(15, true);
        shoulderMotor.setConversionFactor(SHOULDER_CONVERSION_FACTOR);
        shoulderMotor.setConversionFactor(TELESCOPE_CONVERSION_FACTOR);
        shoulderEncoder.configAllSettings(_canCoderConfiguration);
        beltoMotor = new PWMSparkMax(0);
    }

    public void extend() { //arm telescopes out
        //limit extension distance
        NtHelper.setBoolean("/robot/telescope/outness",true);

        if(telescopeMotor.getDistance() >= TELESCOPE_MAXIMUM) {
            telescopeMotor.setPercent(0); 
        } else {
            telescopeMotor.setPercent(TELESCOPE_EXTENSION_POWER);
        }
    }

    public void retract() { //arm un-telescopes
        //limit retraction distance
        NtHelper.setBoolean("/robot/telescope/outness",false);
        if(telescopeMotor.getDistance() <= TELESCOPE_MINIMUM){
            telescopeMotor.setPercent(0); 
        } else {
            telescopeMotor.setPercent(TELESCOPE_RETRACTION_POWER);
        }
    
    }
    public void telescope_override() {
        telescopeMotor.setPercent(TELESCOPE_RETRACTION_POWER%2);
    } 
    public void telescopeToSetpoint(double meters) {
       // setpoint is in meters instead of inches or an encoder value (subject to change)
       NtHelper.setDouble("/robot/telescope/desired_pos", meters);
        if (telescopeMotor.getDistance() <= TELESCOPE_MINIMUM || telescopeMotor.getDistance() >= TELESCOPE_MAXIMUM) {
            NtHelper.setBoolean("/robot/telescope/moving", false);
            telescopeMotor.setPercent(0); 
        } else {
            NtHelper.setBoolean("/robot/telescope/moving", true);
            telescopeMotor.setDistance(meters);
        }
    }

    public double getTelescopePosition() {
        NtHelper.setDouble("/robot/telescope/dist", telescopeMotor.getDistance());
        return telescopeMotor.getDistance();
       
    }

    public void resetTelescopeEncoder() {
        telescopeMotor.resetEncoder(0);

    }

    public void stopTelescopeMotor () {
        telescopeMotor.setPercent(0);
    }

    //moves the shoulder angle towards the front of the robot
    //limit shoulder angle
    public void shoulderForward() {
         if (shoulderEncoder.getPosition() >= SHOULDER_MAXIMUM) {
            NtHelper.setDouble("/robot/shoulder/speed", 0);

            set_shoulder_dist(currentShoulderAngle); /*setting it to the current angle 
            instead of setting percent to 0 because that stops the motor which in turn makes
            the arm fall (not fun)*/ 
        } else {
            NtHelper.setDouble("/robot/shoulder/speed", SHOULDER_FORWARD_POWER);
            set_shoulder_dist(currentShoulderAngle + 1); 
            currentShoulderAngle = shoulderEncoder.getPosition() + 1;
        }
    }

    //moves the shoulder angle towards the back of the robot
    //limit shoulder angle
    public void shoulderBack() {
        if (shoulderEncoder.getPosition() <= SHOULDER_MAXIMUM) {
            set_shoulder_dist(currentShoulderAngle);
            NtHelper.setDouble("/robot/shoulder/speed", 0);
        } else {
            set_shoulder_dist(currentShoulderAngle - 1);
            currentShoulderAngle = shoulderEncoder.getPosition() - 1;
            NtHelper.setDouble("/robot/shoulder/speed", SHOULDER_BACKWARD_POWER);
        }
    }

    public void shoulderStop(){
        set_shoulder_dist_PID(currentShoulderAngle);
        NtHelper.setDouble("/robot/shoulder/speed", 0);
    }
    // this function is to give a certain degrees int and this function will set the motor to the desired location.

    private void set_shoulder_dist(double degrees) {
        NtHelper.setDouble("/robot/shoulder/desired", degrees);
        if(Math.abs(shoulderEncoder.getPosition() - degrees) < 5) {
            shoulderMotor.setPercent(0);
            NtHelper.setDouble("/robot/shoulder/speed", 0);
        }
        else  if (shoulderEncoder.getPosition() > degrees) {
            shoulderMotor.setPercent(SHOULDER_FORWARD_POWER);
            NtHelper.setDouble("/robot/shoulder/speed", SHOULDER_FORWARD_POWER);
        }
        else {
             shoulderMotor.setPercent(-SHOULDER_FORWARD_POWER);
             NtHelper.setDouble("/robot/shoulder/speed", -SHOULDER_FORWARD_POWER);
        }

    } 

    public void set_shoulder_dist_PID(double degrees) {
        shoulderMotor.setPercent(0.3 * -shoulder_PID.calculate(shoulderEncoder.getPosition(), degrees));
       //bleh
    }

    public void shoulderSetpoint(Rotation2d shoulderAngle) {
        if(shoulderEncoder.getPosition() <= SHOULDER_MAXIMUM && shoulderEncoder.getPosition() >= SHOULDER_MINIMUM) {
            set_shoulder_dist_PID(shoulderAngle.getDegrees());
            currentShoulderAngle = shoulderAngle.getDegrees();
        } else{
            set_shoulder_dist(currentShoulderAngle);
        }
        
    }

    public void reset_shoulder () {
         shoulderEncoder.setPosition(0);

    }

    public void getShoulderEncoderCANErrors(){
        String error = shoulderEncoder.getLastError().toString();
        NtHelper.setString("/robot/shoulder/error", error);
    }

    public boolean correctCANError(){
        //do correcting
        //bleh bleh bleeh blehh
        if (shoulderEncoder.getLastError().toString() == ""){
            return true;
        } else {
            return false;
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
        //very psudo code stuff for the belt
        beltoMotor.set(beltoSpeedo);
    }

    public void outtakeBelt() {
        //sets belt speed to # < 0
        beltoMotor.set(-beltoSpeedo);
    }

    public void beltStop() {
        //set belt speed to 0
        beltoMotor.set(0);
    }

    public void setBeltSpeed(double beltoSpeed) {
        //sets constants for belt speeds for intakeBelt, outtakeBelt, and beltStop
        beltoSpeedo = beltoSpeed;
    }
    
   
    public Rotation2d getShoulderAngle() { //may update later
        NtHelper.setDouble("/robot/shoulder/angle", shoulderEncoder.getPosition());
        return new Rotation2d(shoulderEncoder.getPosition());
    }
}
