package frc.robot;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.util.NtHelper;

public class Arm {
    private NeoMotor telescopeMotor;
    private NeoMotor shoulderMotor;
    private PWMSparkMax beltoMotor;
    private static final int TELESCOPE_MOTOR_CAN_BUS_PORT = 9;
    private static final double TELESCOPE_EXTENSION_POWER = 0.5;
    private static final double TELESCOPE_RETRACTION_POWER = -TELESCOPE_EXTENSION_POWER;
    private static final double SHOULDER_FORWARD_POWER = 0.005;
    public static final double DISTANCE = 0;
    private double SHOULDER_CONVERSION_FACTOR = 1; // Calculate later (motor is 80:1)
    private double SHOULDER_MINIMUM = -110; // calculate later ;)
    private double SHOULDER_MAXIMUM = 110; // calculate later :)
    private double TELESCOPE_MINIMUM = 0;
    private double TELESCOPE_MAXIMUM = 97;
    private CANCoder shoulderEncoder = new CANCoder(25);
    CANCoderConfiguration _canCoderConfiguration = new CANCoderConfiguration();
    PIDController shoulder_PID = new PIDController(.005, 0, 0);
    private double TELESCOPE_CONVERSION_FACTOR = 1; // gear ratio
    private double beltoSpeedo = .5;
    private double outtakeBeltoSpeedo = -0.35;
    // Create a new ArmFeedforward with gains kS, kG, kV, and kA
    private ArmFeedforward feedforward = new ArmFeedforward(0.16623, 0.39399, 17.022, 1.7561);
   // private PneumaticHub hub = new PneumaticHub(1);
   // private DoubleSolenoid gripper = hub.makeDoubleSolenoid( 1, 0);
    public DoubleSolenoid gripper = new DoubleSolenoid(22,PneumaticsModuleType.REVPH, 1, 0);
    private double shoulderVoltage = 0;
    public static final double MAX_SHOULDER_VOLTAGE = 4;
    private Rotation2d shoulderSetpoint = new Rotation2d();

    public Arm() {
        // constructs stuff
        telescopeMotor = new NeoMotor(TELESCOPE_MOTOR_CAN_BUS_PORT, false);
        telescopeMotor.setPid(.1, 0.0000, 0);

        shoulderMotor = new NeoMotor(15, true);
        // TODO: These conversion factors need to be fixed
        shoulderMotor.setConversionFactor(SHOULDER_CONVERSION_FACTOR);
        shoulderMotor.setConversionFactor(TELESCOPE_CONVERSION_FACTOR);
        _canCoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        _canCoderConfiguration.magnetOffsetDegrees = -78;
        shoulderEncoder.configAllSettings(_canCoderConfiguration);
        beltoMotor = new PWMSparkMax(0);
        shoulder_PID.setTolerance(5);
    }

    public void extend() { // arm telescopes out
        // limit extension distance
        if (telescopeMotor.getDistance() >= TELESCOPE_MAXIMUM) {
            telescopeMotor.setPercent(0);
        } else {
            telescopeMotor.setPercent(TELESCOPE_EXTENSION_POWER);
        }
    }

    public void retract() { // arm un-telescopes
        // limit retraction distance
        telescopeMotor.setPercent(TELESCOPE_RETRACTION_POWER);

        // if (telescopeMotor.getDistance() <= TELESCOPE_MINIMUM) {
        //     telescopeMotor.setPercent(0);
        // } else {
        //     telescopeMotor.setPercent(TELESCOPE_RETRACTION_POWER);
        // }

    }

    public void telescopeToSetpoint(double meters) {
        // setpoint is in meters instead of inches or an encoder value (subject to
        // change)
        telescopeMotor.setDistance(meters);

        // if (telescopeMotor.getDistance() <= TELESCOPE_MINIMUM || telescopeMotor.getDistance() >= TELESCOPE_MAXIMUM) {
        //     telescopeMotor.setPercent(0);
        // } else {
        //     telescopeMotor.setDistance(meters);
        // }
    }

    public double getTelescopePosition() {
        return telescopeMotor.getDistance();
    }

    public void resetTelescopeEncoder() {
        telescopeMotor.resetEncoder(0);

    }

    public void stopTelescopeMotor() {
        telescopeMotor.setPercent(0);
    }

    // moves the shoulder angle towards the front of the robot
    // limit shoulder angle
    public void shoulderForward() {
        setShoulderSetpoint(shoulderSetpoint.plus(Rotation2d.fromDegrees(5)));
    }

    // moves the shoulder angle towards the back of the robot
    // limit shoulder angle
    public void shoulderBack() {
        setShoulderSetpoint(shoulderSetpoint.minus(Rotation2d.fromDegrees(5)));
    }

    private void setShoulderVoltage(double voltage) {
        shoulderVoltage = voltage;
    }

    private void setShoulderVelocity(double radiansPerSecond) {
        double voltage = feedforward.calculate(getShoulderAngle().getRadians() + (Math.PI / 2), radiansPerSecond);
        setShoulderVoltage(voltage);
    }

    public void shoulderStop() {
        setShoulderSetpoint(getShoulderAngle());
    }
    // this function is to give a certain degrees int and this function will set the
    // motor to the desired location.

    public void setShoulderSetpoint(Rotation2d shoulderAngle) {
        shoulderSetpoint = shoulderAngle;
    }

    public void resetShoulder() {
        shoulderEncoder.setPosition(0);
    }

    public void getShoulderEncoderCANErrors() {
        String error = shoulderEncoder.getLastError().toString();
        NtHelper.setString("/robot/shoulder/error", error);
    }

    public boolean correctCANError() {
        // do correcting
        // bleh bleh bleeh blehh
        if (shoulderEncoder.getLastError().toString() == "") {
            return true;
        } else {
            return false;
        }
    }

    public void openGripper() {
        // pneumatics (no input)
        gripper.set(DoubleSolenoid.Value.kForward);
    }

    public void closeGripper() {
        // pneumatics (no input)
        gripper.set(DoubleSolenoid.Value.kReverse);
    }

    public void intakeBelt() {
        // sets belt speed to # > 0
        // very psudo code stuff for the belt
        beltoMotor.set(beltoSpeedo);
    }

    public void outtakeBelt() {
        // sets belt speed to # < 0
        beltoMotor.set(outtakeBeltoSpeedo);
    }

    public void beltStop() {
        // set belt speed to 0
        beltoMotor.set(0);
    }

    public void setBeltSpeed(double beltoSpeed) {
        // sets constants for belt speeds for intakeBelt, outtakeBelt, and beltStop
        beltoSpeedo = beltoSpeed;
    }

    public Rotation2d getShoulderAngle() { // may update later
        return Rotation2d.fromDegrees(shoulderEncoder.getAbsolutePosition());
    }

    public double getShoulderEncoderPosition() {
        return shoulderEncoder.getAbsolutePosition();
    }

    private double calculatePid(Rotation2d angle) {
        return -shoulder_PID.calculate(shoulderEncoder.getAbsolutePosition(), angle.getDegrees());
    }

    public void periodic() {

        telemtry();

        setShoulderVelocity(calculatePid(shoulderSetpoint));

        if (shoulderEncoder.getAbsolutePosition() >= SHOULDER_MAXIMUM && shoulderVoltage < 0) {
            shoulderMotor.setPercent(0);
        } else if (shoulderEncoder.getAbsolutePosition() <= SHOULDER_MINIMUM && shoulderVoltage > 0) {
            shoulderMotor.setPercent(0);
        } else {
            var voltage = MathUtil.clamp(shoulderVoltage, -MAX_SHOULDER_VOLTAGE, MAX_SHOULDER_VOLTAGE);
            shoulderMotor.setPercent(voltage / RobotController.getBatteryVoltage());
        }

        if (telescopeMotor.getDistance() <= TELESCOPE_MINIMUM || telescopeMotor.getDistance() >= TELESCOPE_MAXIMUM) {
            telescopeMotor.setPercent(0);
        }
    }

    private void telemtry() {
        NtHelper.setDouble("/arm/telescopeDistance", telescopeMotor.getDistance());
        NtHelper.setDouble("/arm/shoulderAngle", getShoulderAngle().getDegrees());
        NtHelper.setDouble("/arm/beltMotor", beltoMotor.get());
        
    }
}
