package frc.robot;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.util.NtHelper;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.math.controller.PIDController;

public class Arm {
    private PWMSparkMax gripperMotor;
    private NeoMotor telescopeMotor;
    private NeoMotor shoulderMotor;
    private PWMSparkMax beltoMotor;
    private PIDController telescopePIDController = new PIDController(1, 0, 0);
    private static final int GRIPPER_MOTOR_PWM_PORT = 0;
    private static final double GRIPPER_OPEN_MOTOR_POWER = 0.5;
    private static final double GRIPPER_CLOSE_MOTOR_POWER = -GRIPPER_OPEN_MOTOR_POWER;
    private static final int TELESCOPE_MOTOR_CAN_BUS_PORT = 9;
    private static final double TELESCOPE_EXTENSION_POWER = 0.5;
    private static final double TELESCOPE_RETRACTION_POWER = -TELESCOPE_EXTENSION_POWER;
    private static final double SHOULDER_FORWARD_POWER = 0.005;
    private static final double SHOULDER_BACKWARD_POWER = -SHOULDER_FORWARD_POWER;
    public static final double DISTANCE = 0;
    private double SHOULDER_CONVERSION_FACTOR = 1; // Calculate later (motor is 80:1)
    private double SHOULDER_MINIMUM = -110; // calculate later ;)
    private double SHOULDER_MAXIMUM = 110; // calculate later :)
    private double TELESCOPE_MINIMUM = 0;
    private double TELESCOPE_MAXIMUM = 97;
    private CANCoder shoulderEncoder = new CANCoder(25);
    CANCoderConfiguration _canCoderConfiguration = new CANCoderConfiguration();
    PIDController shoulder_PID = new PIDController((Robot.isSimulation()) ? .001 : .005, 0, 0);
    private double TELESCOPE_CONVERSION_FACTOR = 1; // gear ratio
    private double beltoSpeedo = .5;
    private double outtakeBeltoSpeedo = -0.35;
    // Create a new ArmFeedforward with gains kS, kG, kV, and kA
    private ArmFeedforward feedforward = new ArmFeedforward(0.16623, 0.39399, 17.022, 1.7561);

    private double shoulderVoltage = 0;
    private double telescopeVoltage = 0;
    private double telescopeSetPoint = 0;
    private boolean telescopePIDUsage = false;
    public static final double MAX_SHOULDER_VOLTAGE = 4;
    private Rotation2d shoulderSetpoint = new Rotation2d(); // worrrrrrrrrrrrrrrrrrrrrrrrrk

    private MechanismLigament2d shoulder;
    private MechanismLigament2d telescope;

    /*
     * Shoulder Angle
     * telescope distance
     * (and sim but not now)
     */

    private Rotation2d shoulderAngle = new Rotation2d(0);
    private double telescopeDist = 0;

    // private final FlywheelSim shoulderSimMotor = new
    // FlywheelSim(DCMotor.getNEO(1), 6.75, 0.025);
    private final FlywheelSim telescopeSimMotor = new FlywheelSim(DCMotor.getNEO(1), 150.0 / 7.0, 0.004096955);
    private static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;
    
    private final FlywheelSim shoulderSimMotor = new FlywheelSim(DCMotor.getNEO(1), 6.75, 0.025);

    /*
     * TODO:
     * - check if getting can values from shoulder encoder -> if not reduce speed ->
     * do later
     * - sleep (more than 5 hours)
     * - bleh
     */

    public Arm() {
        // constructs stuff
        // gripperMotor = new PWMSparkMax(GRIPPER_MOTOR_PWM_PORT);
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
        NtHelper.setDouble("/sim/shoulderAngle", 0);
        NtHelper.setDouble("/sim/telescopeDist", 0);

        // the main mechanism object
        Mechanism2d mech = new Mechanism2d(3, 3);
        // the mechanism root node
        MechanismRoot2d arm = mech.getRoot("arm", 2, .5);

        // MechanismLigament2d objects represent each "section"/"stage" of the
        // mechanism, and are based
        // off the root node or another ligament object
        shoulder = arm.append(
                new MechanismLigament2d("shoulder", Units.inchesToMeters(21), 0, 10, new Color8Bit(Color.kOrange)));
        telescope = shoulder.append(
                new MechanismLigament2d("telescope", 0, 0, 6, new Color8Bit(Color.kGreen)));

        // post the mechanism to the dashboard
        SmartDashboard.putData("Mech2d", mech);
        shoulderSimMotor.setInput(0);

    }

    public void extend() { // arm telescopes out
        // limit extension distance
        NtHelper.setBoolean("/robot/telescope/outness", true);
        telescopePIDUsage = false;

        if (telescopeDist >= TELESCOPE_MAXIMUM) {
            telescopeMotor.setPercent(0);
        } else {
            telescopeMotor.setPercent(TELESCOPE_EXTENSION_POWER);
        }
        if (Robot.isSimulation()) {
            telescopeVoltage = TELESCOPE_EXTENSION_POWER * 12;

        }
    }

    public void retract() { // arm un-telescopes
        // limit retraction distance
        telescopeMotor.setPercent(TELESCOPE_RETRACTION_POWER);
        telescopePIDUsage = false;

        // NtHelper.setBoolean("/robot/telescope/outness", false);
        // if (telescopeDist <= TELESCOPE_MINIMUM) {
        // telescopeMotor.setPercent(0);
        // } else {
        // telescopeMotor.setPercent(TELESCOPE_RETRACTION_POWER);
        // }
        if (Robot.isSimulation()) {
            telescopeVoltage = TELESCOPE_RETRACTION_POWER * 12;
            NtHelper.setBoolean("/arm/retract", true);
        }
    }

    public void telescopeToSetpoint(double meters) {
        // setpoint is in meters instead of inches or an encoder value (subject to
        // change)
        // telescopeMotor.setDistance(meters);
        //deletelescopeVoltage = telescopePIDController.calculate(telescopeDist, meters);
        // telescopeMotor.setPercent(telescopeVoltage / RobotController.getBatteryVoltage());
        // System.out.println("Set");
        // telescopeSimMotor.setInputVoltage(telescopeVoltage);
        NtHelper.setBoolean("/sim/telescopeToSetpoint", true);
        telescopePIDUsage = true;
        telescopeSetPoint = meters;
        // NtHelper.setDouble("/robot/telescope/desired_pos", meters);
        // if (telescopeDist <= TELESCOPE_MINIMUM || telescopeDist >= TELESCOPE_MAXIMUM)
        // {
        // NtHelper.setBoolean("/robot/telescope/moving", false);
        // telescopeMotor.setPercent(0);
        // } else {
        // NtHelper.setBoolean("/robot/telescope/moving", true);
        // telescopeMotor.setDistance(meters);
        // }
    }

    public double getTelescopePosition() {
        NtHelper.setDouble("/robot/telescope/dist", telescopeDist);
        return telescopeDist;

    }

    public void resetTelescopeEncoder() {
        telescopeMotor.resetEncoder(0);

    }

    public void stopTelescopeMotor() {
        telescopeMotor.setPercent(0);
        telescopeVoltage = 0;
        telescopePIDUsage = false;
    }



    // moves the shoulder angle towards the front of the robot
    // limit shoulder angle
    public void shoulderForward() {
        NtHelper.setDouble("/robot/shoulder/speed", SHOULDER_FORWARD_POWER);
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
        NtHelper.setDouble("/robot/shoulder/speed", 0);
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
    }

    public void closeGripper() {
        // pneumatics (no input)
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
        if (Robot.isSimulation()) {
            return shoulderAngle;
        }
        return Rotation2d.fromDegrees(shoulderEncoder.getAbsolutePosition());
    }

    public double getShoulderEncoderPosition() {
        return shoulderEncoder.getAbsolutePosition();
    }

    private double calculatePid(Rotation2d angle) {
        return shoulder_PID.calculate(shoulderAngle.getDegrees(), angle.getDegrees());
    }

    public void simPeriodic(double dtSeconds, double shoulderMotorPercent, double telescopeMotorPercent) {
        // Update simulation inputs
        NtHelper.setDouble("/sim/telescopeMotorPercent", telescopeMotorPercent);

        shoulderSimMotor.setInputVoltage(shoulderMotorPercent * RobotController.getBatteryVoltage());
        telescopeSimMotor.setInputVoltage(telescopeMotorPercent * RobotController.getBatteryVoltage());
        // System.out.println(shoulderSimMotor.getAngularVelocityRPM());

        // Move simulation forward dt seconds
        shoulderSimMotor.update(dtSeconds);
        telescopeSimMotor.update(dtSeconds);
        var encoderRateSign = 1;

        // Get state from simulation devices (telescopeDist and shoulderAngle)
        var telescopeRate = telescopeSimMotor.getAngularVelocityRadPerSec() * encoderRateSign;
        telescopeDist += telescopeRate * 0.02;
        var shoulderRate = shoulderSimMotor.getAngularVelocityRadPerSec() * encoderRateSign;
        shoulderAngle = shoulderAngle.plus(Rotation2d.fromRadians(shoulderRate * 0.02));
        // NtHelper.setDouble("/sim/telescopeRate", telescopeRate);
        // NtHelper.setDouble("/sim/shoulderRate", shoulderRate);
        NtHelper.setDouble("/sim/shoulderVolt", shoulderMotorPercent * RobotController.getBatteryVoltage());
        NtHelper.setDouble("/sim/shoulderAngleDesired", shoulderSetpoint.getDegrees());
        NtHelper.setDouble("/sim/shoulderAngleMeasured", shoulderAngle.getDegrees());

        // mechanism2d
        telescope.setLength(telescopeDist / 100);
        shoulder.setAngle(shoulderAngle.getDegrees() + 90);

        NtHelper.setDouble("/sim/telescopeMotorPercent", telescopeMotorPercent);
    }

    public void realPeriodic(double shoulderMotorPercent, double telescopeMotorPercent) {
        // Update real robot inputs
        setShoulderVelocity(calculatePid(shoulderSetpoint));
        if (shoulderAngle.getDegrees() >= SHOULDER_MAXIMUM && shoulderVoltage < 0) {
            shoulderMotor.setPercent(0);
        } else if (shoulderAngle.getDegrees() <= SHOULDER_MINIMUM && shoulderVoltage > 0) {
            shoulderMotor.setPercent(0);
        } else {
            var voltage = MathUtil.clamp(shoulderVoltage, -MAX_SHOULDER_VOLTAGE, MAX_SHOULDER_VOLTAGE);
            shoulderMotor.setPercent(voltage / RobotController.getBatteryVoltage());
        }

        // Update state from actual devices (telescopeDist and shoulderAngle)
        if (telescopeDist <= TELESCOPE_MINIMUM || telescopeDist >= TELESCOPE_MAXIMUM) {
            NtHelper.setBoolean("/robot/telescope/moving", false);
            telescopeMotor.setPercent(0);
        }
    }

    public void periodic() {
        telemtry();
        double shoulderMotorPercent = shoulderMotor.getPercent();
        double telescopeMotorPercent = (telescopeVoltage / RobotController.getBatteryVoltage());
        if (telescopePIDUsage){
            telescopeMotorPercent = telescopePIDController.calculate(telescopeDist, telescopeSetPoint) / RobotController.getBatteryVoltage();
        }

        setShoulderVelocity(calculatePid(shoulderSetpoint));
        if (shoulderAngle.getDegrees() >= SHOULDER_MAXIMUM && shoulderVoltage < 0) {
            shoulderMotorPercent = 0;
        } else if (shoulderAngle.getDegrees() <= SHOULDER_MINIMUM && shoulderVoltage > 0) {
            shoulderMotorPercent = 0;
        } else {
            var voltage = MathUtil.clamp(shoulderVoltage, -MAX_SHOULDER_VOLTAGE, MAX_SHOULDER_VOLTAGE);
            shoulderMotorPercent = (voltage / RobotController.getBatteryVoltage());
        }

        // Update state from actual devices (telescopeDist and shoulderAngle)
        // if (telescopeDist <= TELESCOPE_MINIMUM || telescopeDist >= TELESCOPE_MAXIMUM) {
        //     NtHelper.setBoolean("/robot/telescope/moving", false);
        //     telescopeMotorPercent = 0;
        // }

        if (Robot.isSimulation()) {
            NtHelper.setDouble("/sim/shoulderSetpoint", shoulderSetpoint.getDegrees());
            simPeriodic(0.02, shoulderMotorPercent, telescopeMotorPercent);
        } else {

            realPeriodic(shoulderMotorPercent, telescopeMotorPercent);

        }
    }

    private void telemtry() {
        NtHelper.setDouble("/arm/telescopeDistance", telescopeDist);
        NtHelper.setDouble("/arm/shoulderAngle", getShoulderAngle().getDegrees());
        NtHelper.setDouble("/arm/beltMotor", beltoMotor.get());
    }
}
