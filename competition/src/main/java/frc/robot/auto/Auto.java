package frc.robot.auto;

import frc.robot.util.stateMachine.State;
import frc.robot.util.stateMachine.StateContext;
import frc.robot.util.stateMachine.StateMachine;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Auto extends StateMachine {

    StateMachine selectedAutonomous;
    StateMachine taxi;
    StateMachine yoyo;
    StateMachine testauto;
    StateMachine balanceauto;
    StateMachine gyroauto;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    public Auto() {
        super("run");
        taxi = new BasedAuto();
        yoyo = new YoYoAuto();
        testauto = new TestAuto();
        balanceauto = new GyroAuto();
        gyroauto = new GyroAuto();

        m_chooser.setDefaultOption("taxi", "taxi");
        m_chooser.addOption("yoyo", "yoyo");
        m_chooser.addOption("testauto", "testauto");
        m_chooser.addOption("balanceauto", "balanceauto");
        m_chooser.addOption("gyroauto", "gyroauto");

        SmartDashboard.putData("Auto choices", m_chooser);
    }

    public void getAuto() {
        String name = m_chooser.getSelected();


        switch (name) {
            case "taxi":
                selectedAutonomous = taxi;
                break;
            case "yoyo":
                selectedAutonomous = yoyo;
                break;
            case "testauto":
                selectedAutonomous = testauto;
                break;
            case "balanceauto":
                selectedAutonomous = balanceauto;
                break;
            case "gyroauto":
                selectedAutonomous = gyroauto;
                break;
            default:
                selectedAutonomous = taxi;
                break;
        }
    }

    public void restart() {
        setState("run");
    }

    @State(name = "run")
    public void runState(StateContext ctx) {
        if (ctx.isInit()) {
            getAuto();
            selectedAutonomous.setState(selectedAutonomous.getDefaultState());
        }
        selectedAutonomous.run();
    }
}
