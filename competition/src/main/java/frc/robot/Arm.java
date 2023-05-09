package frc.robot;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.apache.commons.collections4.multimap.HashSetValuedHashMap;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.util.NtHelper;
import frc.robot.util.RateChecker;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Arm {

    public enum Position  {
        floor,
        low,
        mid,
        high,
        HP
    }

    public enum RobotPart {
        shoulder,
        telescope,
        wrist
    }

    private static HashMap<Position, HashMap<RobotPart, Double>> setpoints = new HashMap<>();

    static {
        setpoints.put(Position.floor, new HashMap<>());
        setpoints.put(Position.low, new HashMap<>());
        setpoints.put(Position.mid, new HashMap<>());
        setpoints.put(Position.high, new HashMap<>());
        setpoints.put(Position.HP, new HashMap<>());

    }

    static {
        setpoints.get(Position.floor).put(RobotPart.shoulder , 110.0);
        setpoints.get(Position.floor).put(RobotPart.telescope , 10.0);
        setpoints.get(Position.floor).put(RobotPart.wrist , 30.0);

        setpoints.get(Position.low).put(RobotPart.shoulder , 90.0);
        setpoints.get(Position.low).put(RobotPart.telescope , 10.0);
        setpoints.get(Position.low).put(RobotPart.wrist , 30.0);

        setpoints.get(Position.mid).put(RobotPart.shoulder , 90.0);
        setpoints.get(Position.mid).put(RobotPart.telescope , 10.0);
        setpoints.get(Position.mid).put(RobotPart.wrist , 30.0);

        setpoints.get(Position.high).put(RobotPart.shoulder , 90.0);
        setpoints.get(Position.high).put(RobotPart.telescope , 10.0);
        setpoints.get(Position.high).put(RobotPart.wrist , 30.0);

        setpoints.get(Position.HP).put(RobotPart.shoulder , 90.0);
        setpoints.get(Position.HP).put(RobotPart.telescope , 10.0);
        setpoints.get(Position.HP).put(RobotPart.wrist , 30.0);

    }

    private NeoMotor telescopeMotor;
    private NeoMotor shoulderMotor;
    private NeoMotor beltoMotor;
    private NeoMotor wristoMotor;
    private ProfiledPIDController telescopePIDController = new ProfiledPIDController(0.7, 0, 0, new TrapezoidProfile.Constraints(60, 80));
    private static final int TELESCOPE_MOTOR_CAN_BUS_PORT = 9;
    public static final double DISTANCE = 0;
    private double SHOULDER_MAXIMUM = 125; // calculate later :)
    private double TELESCOPE_MINIMUM = 0;
    private double WRIST_MAXIMUM = 0; // calculate later :)
    private double TELESCOPE_MAXIMUM = 30;//97;
    private CANCoder shoulderEncoder = new CANCoder(25);
    private CANCoder wristEncoder = new CANCoder();//set deviceNumber
    CANCoderConfiguration _canCoderConfiguration = new CANCoderConfiguration();
    ProfiledPIDController shoulder_PID = new ProfiledPIDController((Robot.isSimulation()) ? .001 : .005, 0, 0, new TrapezoidProfile.Constraints(360, 420));//noice
    ProfiledPIDController wrist_PID = new ProfiledPIDController((Robot.isSimulation()) ? 0 : 0, 0, 0, new TrapezoidProfile.Constraints(0, 0));//PLEASE SET VALUES

    private double TELESCOPE_CONVERSION_FACTOR = 1; // gear ratio
    private double beltoSpeedo = 0.8;
    private double outtakeBeltoSpeedo = -1;
    // Create a new ArmFeedforward with gains kS, kG, kV, and kA
    private final double kg = RobotBase.isSimulation() ? 0 : 0.39399;
    private ArmFeedforward feedforward = new ArmFeedforward(0.16623, kg, 17.022, 1.7561);
    public DoubleSolenoid gripper = new DoubleSolenoid(22, PneumaticsModuleType.REVPH, 1, 0);
    private double shoulderVoltage = 0;
    private double telescopeVoltage = 0;
    private double telescopeSetPoint = 0;
    public static final double MAX_SHOULDER_VOLTAGE = 4;
    private Rotation2d shoulderSetpoint = new Rotation2d();
    private Rotation2d wristSetpoint = new Rotation2d();

    private MechanismLigament2d shoulder;
    private MechanismLigament2d telescope;

    private Rotation2d shoulderAngle = new Rotation2d(0);
    private Rotation2d wristAngle = new Rotation2d(0);

    private double telescopeDist = 0;

    private final FlywheelSim telescopeSimMotor = new FlywheelSim(DCMotor.getNEO(1), 150.0 / 7.0, 0.004096955);

    private final FlywheelSim shoulderSimMotor = new FlywheelSim(DCMotor.getNEO(1), 6.75, 0.025);

    private boolean isSafeMode = true;

    private RateChecker telescopeRateChecker = new RateChecker(0.5);

    public Arm() {
        // constructs stuff
        telescopeMotor = new NeoMotor(TELESCOPE_MOTOR_CAN_BUS_PORT, false);
        telescopeMotor.setPid(.1, 0.0000, 0);

        shoulderMotor = new NeoMotor(15, true);
        // TODO: These conversion factors need to be fixed
        // shoulderMotor.setConversionFactor(TELESCOPE_CONVERSION_FACTOR);
        _canCoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        _canCoderConfiguration.magnetOffsetDegrees = -135;
        shoulderEncoder.configAllSettings(_canCoderConfiguration);
        beltoMotor = new NeoMotor(0); //PLEASE SET PORT thx :P
        shoulder_PID.setTolerance(RobotBase.isSimulation() ? 5 : 5);
        NtHelper.setDouble("/sim/shoulderAngle", 0);
        NtHelper.setDouble("/sim/telescopeDist", 0);

        // the main mechanism object
        Mechanism2d mech = new Mechanism2d(3, 3);
        // the mechanism root node
        MechanismRoot2d arm = mech.getRoot("arm", 1.5, .5);

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
        telescopeToSetpoint(telescopeSetPoint + 3 * 0.02); // 0.02 subject to change
    }

    public void retract() { // arm un-telescopes
        // limit retraction distance
        
        if (isSafeMode) {
    
            telescopeToSetpoint(telescopeSetPoint - 3 * 0.02); 
        }
        else {
            telescopeToSetpoint(telescopeSetPoint - 6 * 0.02); 
        }
    }

    public void telescopeToSetpoint(double meters) {
        // setpoint is in meters instead of inches or an encoder value (subject to
        // change)
        NtHelper.setBoolean("/sim/telescopeToSetpoint", true);
        if (isSafeMode) {
            telescopeSetPoint = Math.max(0, meters);
        } else {
            telescopeSetPoint = meters;
        }
    }

    public double getTelescopePosition() {
        return telescopeDist;
    }

    public void resetTelescopeEncoder() {
        telescopeMotor.resetEncoder(0);
    }

    public void stopTelescopeMotor() {
        telescopeSetPoint = telescopeDist;
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

    public void setShoulderSetpoint(Rotation2d shoulderAngle) { // to spin comment out the if statement around ahoulder
                                                                // setpoint stuff
        if (shoulderAngle.getDegrees() >= -SHOULDER_MAXIMUM && shoulderAngle.getDegrees() <= SHOULDER_MAXIMUM) {
            shoulderSetpoint = shoulderAngle;
        }
    }

    public void resetShoulder() {
        shoulderEncoder.setPosition(0);
    }

    public void setPosition(Position setpointValue, boolean isCubes) {
        //if the shoulder is less than 90 degrees keep moving the wrist, 
        //otherwise stop the shoulder and let the wrist finsh moving.
        setShoulderSetpoint(shoulderSetpoint.plus(Rotation2d.fromDegrees(setpoints.get(setpointValue).get(RobotPart.shoulder))));
        telescopeToSetpoint(setpoints.get(setpointValue).get(RobotPart.telescope));
        wristToSetpoint(shoulderSetpoint.plus(Rotation2d.fromDegrees(setpoints.get(setpointValue).get(RobotPart.shoulder))));

        //do if isCubes :>
    }

    public void getShoulderEncoderCANErrors() {
        String error = shoulderEncoder.getLastError().toString();
        NtHelper.setString("/robot/shoulder/error", error);
    }

    public boolean correctCANError() {
        // do correcting
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
        beltoMotor.setSpeed(beltoSpeedo); 
    }

    public void outtakeBelt() {
        // sets belt speed to # < 0
        if (NtHelper.getBoolean("/dashboard/arm/isCubes", false)) {
            beltoMotor.setSpeed(outtakeBeltoSpeedo);
        } else {
            beltoMotor.setSpeed(-0.6);
        }
    }
    public void outtakeBeltCube() {
        beltoMotor.setSpeed(outtakeBeltoSpeedo);
    }

    public void outtakeBeltCone() {
        beltoMotor.setSpeed(-0.6);
    }

    public void setOutakeSpeed(double speed) {
        outtakeBeltoSpeedo = speed;
    }

    public void beltStop() {
        // set belt speed to 0
        beltoMotor.setSpeed(0);
        outtakeBeltoSpeedo = -1;
    }

    public void setBeltSpeed(double beltoSpeed) {
        // sets constants for belt speeds for intakeBelt, outtakeBelt, and beltStop
        beltoSpeedo = beltoSpeed;
    }

    public void wristToSetpoint(Rotation2d wristAngle) {
        if (wristAngle.getDegrees() >= -WRIST_MAXIMUM && wristAngle.getDegrees() <= WRIST_MAXIMUM) {
            wristSetpoint = wristAngle;   
        }
    } 

    public void resetWrist() {
        wristEncoder.setPosition(0);
    }

    public Rotation2d getWristAngle() {
        return wristAngle;
    }

    private double wristCalcPID(Rotation2d angle) {
        return wrist_PID.calculate(wristAngle.getDegrees(), angle.getDegrees());
    }

    public double getWristEncoderPosition() {
        return wristEncoder.getAbsolutePosition();
    }

    public Rotation2d getShoulderAngle() {
        return shoulderAngle;
    }

    public double getShoulderEncoderPosition() {
        return shoulderEncoder.getAbsolutePosition();
    }

    private double shoulderCalcPID(Rotation2d angle) {
        return shoulder_PID.calculate(shoulderAngle.getDegrees(), angle.getDegrees());
    }

    public void simPeriodic(double dtSeconds, double shoulderMotorPercent, double telescopeMotorPercent) {
        // Update simulation inputs
        shoulderSimMotor.setInputVoltage(shoulderMotorPercent * RobotController.getBatteryVoltage());
        telescopeSimMotor.setInputVoltage(telescopeMotorPercent * RobotController.getBatteryVoltage());

        // Move simulation forward dt seconds
        shoulderSimMotor.update(dtSeconds);
        telescopeSimMotor.update(dtSeconds);
        var encoderRateSign = 1;

        // Get state from simulation devices (telescopeDist and shoulderAngle)
        var telescopeRate = telescopeSimMotor.getAngularVelocityRadPerSec() * encoderRateSign;
        telescopeDist += telescopeRate * 0.02;
        var shoulderRate = shoulderSimMotor.getAngularVelocityRadPerSec() * encoderRateSign;
        shoulderAngle = shoulderAngle.plus(Rotation2d.fromRadians(shoulderRate * 0.02));

        // mechanism2d :/
        telescope.setLength(telescopeDist / 25);
        shoulder.setAngle(-shoulderAngle.getDegrees() + 90);

    }

    public void isSafeMode(boolean safeMode) {
        isSafeMode = safeMode;
    }

    public void realPeriodic(double shoulderMotorPercent, double telescopeMotorPercent) {
        // Update real robot inputs
        shoulderMotor.setPercent(-shoulderMotorPercent);
        telescopeMotor.setPercent(telescopeMotorPercent);

        telescopeDist = telescopeMotor.getDistance();
        shoulderAngle = Rotation2d.fromDegrees(shoulderEncoder.getAbsolutePosition());
    }

    public void periodic() {
        telemtry();
        double shoulderMotorPercent = shoulderMotor.getPercent();
        double telescopeMotorPercent = (telescopeVoltage / RobotController.getBatteryVoltage());
        telescopeMotorPercent = telescopePIDController.calculate(telescopeDist, telescopeSetPoint)
                / RobotController.getBatteryVoltage();
        setShoulderVelocity(shoulderCalcPID(shoulderSetpoint));
        if (shoulderAngle.getDegrees() >= getMaxShoulderAngle() && shoulderVoltage > 0) {
            shoulderMotorPercent = 0;
        } else if (shoulderAngle.getDegrees() <= -getMaxShoulderAngle() && shoulderVoltage < 0) {
            shoulderMotorPercent = 0;
        } else {
            var voltage = MathUtil.clamp(shoulderVoltage, -MAX_SHOULDER_VOLTAGE, MAX_SHOULDER_VOLTAGE);
            shoulderMotorPercent = (voltage / RobotController.getBatteryVoltage());
        }

        if (telescopeMotorPercent < 0) {
            telescopeRateChecker.update(telescopeDist);
            if (telescopeMotor.getPercent() >= 0) {
                telescopeRateChecker.startTimer();
            }

            var rate = telescopeRateChecker.getRate();

            if (rate != null && Math.abs(rate) < 1) {
                // resetTelescopeEncoder();
            }
        }

        if (isSafeMode && telescopeDist < TELESCOPE_MINIMUM && telescopeMotorPercent < 0) {
            telescopeMotorPercent = 0;
        }

        if (telescopeDist > TELESCOPE_MAXIMUM && telescopeMotorPercent > 0) {
            telescopeMotorPercent = 0;
        }

        if (telescopeMotorPercent < 0 && isSafeMode) {
            telescopeMotorPercent = telescopeMotorPercent * 0.5;
        }

        

        NtHelper.setDouble("/robot/telescopeMotorPercent", telescopeMotorPercent);
        NtHelper.setDouble("/robot/shoulderMotorPercent", shoulderMotorPercent);

        if (Robot.isSimulation()) {
            NtHelper.setDouble("/sim/shoulderSetpoint", shoulderSetpoint.getDegrees());
            simPeriodic(0.02, shoulderMotorPercent, telescopeMotorPercent);
        } else {
            realPeriodic(shoulderMotorPercent, telescopeMotorPercent);
        }
    }

    public double getMaxShoulderAngle() {
        if (telescopeDist > 2){
            return 60;
        } else {
            return SHOULDER_MAXIMUM;
        }

    }
    public Rotation2d getShoulderSetpoint() {
        return shoulderSetpoint;
    }

    public double getTelescopeSetpoint() {
        return telescopeSetPoint;
    }

    public void setEnabled(boolean isEnabled) {
        shoulderMotor.setEnabled(isEnabled);
        telescopeMotor.setEnabled(isEnabled);
    }

    private void telemtry() {
        NtHelper.setDouble("/arm/telescopeDistance", telescopeDist);
        NtHelper.setDouble("/arm/shoulderAngle", getShoulderAngle().getDegrees());
        NtHelper.setDouble("/arm/beltMotor", beltoMotor.getPercent());
    }
}