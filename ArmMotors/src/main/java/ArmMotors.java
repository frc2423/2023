import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class ArmMotors {
    private CANSparkMax arm;
    private CANSparkMax telescope;
    private DoubleSolenoid gripper;
    private double armspeed = 0.5;
    private double telescopespeed = 0.5;
    
    public ArmMotors(){
            arm = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
            telescope = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
            gripper = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 0);
        }

    public void armUp(){
        arm.setVoltage(armspeed);
    }
    public void armDown(){
        arm.setVoltage(-armspeed);
    }
    public void telescopeIn(){
        telescope.setVoltage(telescopespeed);
    }
    public void tlescopeOut(){
        telescope.setVoltage(-telescopespeed);
    }
    public void gripperOpen(){
        gripper.set(DoubleSolenoid.Value.kForward);
    }
    public void gripperClosed(){
        gripper.set(DoubleSolenoid.Value.kReverse);
    }
}
